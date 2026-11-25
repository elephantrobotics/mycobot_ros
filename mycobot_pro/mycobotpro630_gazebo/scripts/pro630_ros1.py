"""rospy transport for the Pro630 controller nodes.

Subscriber and timer callbacks are drained on the owning node's event loop,
including the independent robot proxy. rospy serves RPCs on TCPROS threads.
"""

from collections import deque
from concurrent.futures import Future
import hashlib
import os
import queue
import threading
import time
from types import SimpleNamespace

import rosgraph
import rosnode
import rospkg
import rospy


class DurabilityPolicy:
    TRANSIENT_LOCAL = 'latched'


class HistoryPolicy:
    KEEP_LAST = 'keep_last'


class ReliabilityPolicy:
    RELIABLE = 'reliable'


class QoSProfile:
    def __init__(self, depth, durability=None, history=None, reliability=None):
        self.depth = depth
        self.latched = durability == DurabilityPolicy.TRANSIENT_LOCAL


class SignalHandlerOptions:
    NO = 'manual'


ExternalShutdownException = rospy.ROSInterruptException


class ReentrantCallbackGroup:
    """TCPROS already runs each incoming service connection independently."""


_manual_signals = False


def init(args=None, signal_handler_options=None):
    global _manual_signals
    _manual_signals = signal_handler_options == SignalHandlerOptions.NO


def ok():
    return not rospy.is_shutdown()


def try_shutdown():
    if rospy.core.is_initialized() and ok():
        rospy.signal_shutdown('Pro630 node finished')


shutdown = try_shutdown


def package_directory(package):
    return rospkg.RosPack().get_path(package)


def master_key():
    uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    return hashlib.sha256(uri.encode()).hexdigest()[:16]


def get_logger(_name=None):
    return SimpleNamespace(info=rospy.loginfo, warning=rospy.logwarn,
                           error=rospy.logerr, debug=rospy.logdebug)


class ServiceClient:
    def __init__(self, kind, name, persistent=False):
        self.kind = kind
        self.name = rospy.resolve_name(name)
        self._persistent = persistent
        self._proxy = None
        self._proxy_lock = threading.Lock()
        self._slots = threading.BoundedSemaphore(8)
        self._last_check = float('-inf')
        self._available = False

    def service_is_ready(self):
        now = time.monotonic()
        if now - self._last_check >= .2:
            self._last_check = now
            try:
                rosgraph.Master(rospy.get_name()).lookupService(self.name)
                self._available = True
            except (rosgraph.MasterException, OSError):
                self._available = False
        return self._available

    def wait_for_service(self, timeout_sec):
        if timeout_sec <= 0:
            return self.service_is_ready()
        try:
            rospy.wait_for_service(self.name, timeout=timeout_sec)
            return True
        except rospy.ROSException:
            return False

    def call_async(self, request):
        future = Future()
        if self._persistent:
            # Collision sampling calls this hundreds of times. A thread per
            # sample costs more than the check itself; the reused connection
            # is invoked on the caller's thread.
            try:
                future.set_result(self._invoke(request))
            except Exception as exc:
                future.set_exception(exc)
            return future
        if not self._slots.acquire(False):
            future.set_exception(RuntimeError(f'Too many pending calls to {self.name}'))
            return future

        def call():
            try:
                # A fresh connection lets a STOP query pass another in-flight call.
                # Collision sampling reuses one connection; reconnect if it breaks.
                result = self._invoke(request)
            except Exception as exc:
                if future.set_running_or_notify_cancel():
                    future.set_exception(exc)
            else:
                if future.set_running_or_notify_cancel():
                    future.set_result(result)
            finally:
                self._slots.release()

        threading.Thread(target=call, daemon=True).start()
        return future

    def _invoke(self, request):
        if not self._persistent:
            return rospy.ServiceProxy(self.name, self.kind)(request)
        with self._proxy_lock:
            if self._proxy is None:
                rospy.wait_for_service(self.name, timeout=2.0)
                self._proxy = rospy.ServiceProxy(self.name, self.kind, persistent=True)
            try:
                return self._proxy(request)
            except Exception:
                proxy = self._proxy
                self._proxy = None
                if proxy is not None:
                    try:
                        proxy.close()
                    except Exception:
                        pass
                raise


class Node:
    def __init__(self, name):
        if not rospy.core.is_initialized():
            rospy.init_node(name, disable_signals=_manual_signals)
        self._defaults = {}
        self._handles = []
        self._callbacks = queue.Queue()
        self._destroyed = threading.Event()

    def declare_parameter(self, name, default):
        self._defaults[name] = default

    def get_parameter(self, name):
        return SimpleNamespace(value=rospy.get_param('~' + name, self._defaults[name]))

    def now(self):
        if rospy.get_param('~use_sim_time', rospy.get_param('/use_sim_time', False)):
            return rospy.Time.now()
        return rospy.Time.from_sec(time.time())

    def get_logger(self):
        return get_logger()

    def get_node_names(self):
        return [name.rsplit('/', 1)[-1] for name in rosnode.get_node_names()]

    def create_publisher(self, kind, name, qos):
        depth = qos if isinstance(qos, int) else qos.depth
        pub = rospy.Publisher(name, kind, queue_size=depth, latch=getattr(qos, 'latched', False))
        self._handles.append(pub.unregister)
        return pub

    def _mailbox(self, callback, depth):
        messages = deque(maxlen=depth)
        lock = threading.Lock()

        def dispatch():
            with lock:
                args = messages.popleft()
                if messages:
                    self._callbacks.put(dispatch)
            callback(*args)

        def enqueue(*args):
            with lock:
                if self._destroyed.is_set():
                    return
                if not messages:
                    self._callbacks.put(dispatch)
                messages.append(args)

        return enqueue

    def create_subscription(self, kind, name, callback, qos):
        depth = qos if isinstance(qos, int) else qos.depth
        sub = rospy.Subscriber(name, kind, self._mailbox(callback, depth), queue_size=depth)
        self._handles.append(sub.unregister)
        return sub

    def create_service(self, kind, name, callback, callback_group=None):
        srv = rospy.Service(name, kind, lambda request: callback(request, kind._response_class()))
        self._handles.append(srv.shutdown)
        return srv

    def create_client(self, kind, name, persistent=False):
        return ServiceClient(kind, name, persistent=persistent)

    def create_timer(self, seconds, callback):
        enqueue = self._mailbox(callback, 1)
        cancelled = threading.Event()

        def run():
            while not cancelled.wait(seconds) and ok():
                enqueue()

        worker = threading.Thread(target=run, daemon=True)
        worker.start()

        def stop():
            cancelled.set()
            worker.join(.25)

        self._handles.append(stop)

    def destroy_node(self):
        self._destroyed.set()
        for close in reversed(self._handles):
            close()
        self._handles.clear()


def spin_once(node, timeout_sec=.1):
    try:
        callback = node._callbacks.get(timeout=timeout_sec)
    except queue.Empty:
        return
    if not node._destroyed.is_set():
        callback()


def spin(node):
    while ok() and not node._destroyed.is_set():
        spin_once(node)


def spin_until_future_complete(node, future, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while ok() and not future.done() and time.monotonic() < deadline:
        spin_once(node, min(.05, max(0., deadline - time.monotonic())))


class SingleThreadedExecutor:
    def __init__(self, num_threads=None):
        self._finished = threading.Event()
        self.node = None

    def add_node(self, node):
        self.node = node

    def spin(self):
        while ok() and not self._finished.is_set():
            spin_once(self.node)

    def shutdown(self, timeout_sec=None):
        self._finished.set()


MultiThreadedExecutor = SingleThreadedExecutor
