"""Feature-side ROS proxy. Never imports the hardware SDK or opens a robot socket."""
import json
import math
import threading
import time
import uuid
from contextlib import contextmanager
from types import SimpleNamespace
from pro630_model import ARM_JOINTS, KEYBOARD_ARM_FEEDBACK_MAX_AGE_S
from pro630_broker_support import PublicArmFeedbackUnavailable

import pro630_ros1 as ros
from pro630_ros1 import SingleThreadedExecutor
from pro630_ros1 import Node
from sensor_msgs.msg import JointState
from mycobotpro630_gazebo.srv import Pro630Rpc


class PublicPro630Client:
    public_feedback = True

    def __init__(self, ip=None, netport=None, debug=False, feedback_hz=10.,
                 ownership_timeout=15., feature='slider', with_gripper=True, allow_collision=False):
        self.owner_token = uuid.uuid4().hex
        self.feature = feature
        self._call_lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._generation_lock = threading.Lock()
        self._keyboard_generation = 0
        self._angle_sample = threading.local()
        self._condition = threading.Condition()
        self._callback = None
        self._latest = None
        self._queried_sequence = -1
        self._rpc_sequence = -1
        self._stamps = {}
        self._closed = False
        self._claimed = False
        self._heartbeat_pending = None
        self.node = Node('pro630_feature_proxy_' + self.owner_token[:8])
        self.client = self.node.create_client(Pro630Rpc, '/pro630/robot_rpc')
        self.node.create_subscription(JointState, '/pro630/real_joint_states', self._sample, 1)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        try:
            if not self.client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('Public robot service unavailable; start slider.launch environment:=real')
            from pro630_configuration import joints, require_flag, collision_policy
            self.with_gripper = require_flag(with_gripper)
            self.allow_collision = collision_policy(allow_collision)
            capabilities = self._rpc('get_capabilities')
            if (capabilities.get('with_gripper') is not self.with_gripper or
                    capabilities.get('joints') != joints(self.with_gripper)):
                raise RuntimeError('Public robot installation profile does not match this feature')
            if collision_policy(capabilities.get('allow_collision', False)) is not self.allow_collision:
                raise RuntimeError('Public collision policy does not match this feature; restart the control launch')
            claim = self._rpc('claim', [feature])
            if feature == 'keyboard':
                self._keyboard_generation = claim['generation']
            self._claimed = True
            self.node.create_timer(.5, self._heartbeat)
        except Exception:
            self.close()
            raise

    def _request(self, method, args=(), kwargs=None):
        from pro630_broker_support import KEYBOARD_MOTION_METHODS
        kwargs = dict(kwargs or {})
        if self.feature == 'keyboard' and method in KEYBOARD_MOTION_METHODS:
            with self._generation_lock:
                kwargs['_keyboard_generation'] = getattr(
                    self._angle_sample, 'motion_generation', self._keyboard_generation)
        req = Pro630Rpc._request_class()
        req.token, req.method = self.owner_token, method
        req.args_json = json.dumps(list(args), allow_nan=False)
        req.kwargs_json = json.dumps(kwargs or {}, allow_nan=False)
        return req

    def _rpc(self, method, args=(), kwargs=None):
        # STOP must not queue behind a coordinate/status future.
        guard = self._stop_lock if method == 'keyboard_stop' else self._call_lock
        request = self._request(method, args, kwargs)
        with guard:
            if self._closed:
                raise RuntimeError('Feature proxy closed')
            completed = threading.Event()
            future = self.client.call_async(request)
            future.add_done_callback(lambda _: completed.set())
            from pro630_broker_support import READS
            timeout = 8.0 if self.feature == 'keyboard' else 15.0
            if not completed.wait(timeout):
                future.cancel()
                raise RuntimeError(f'Public robot request timed out: {method}')
            response = future.result()
            if response is None or not response.success:
                from pro630_broker_support import KEYBOARD_CANCELLED, KeyboardCommandCancelled
                if response is not None and response.error == KEYBOARD_CANCELLED:
                    raise KeyboardCommandCancelled()
                raise RuntimeError(response.error if response is not None else 'Public robot service failed')
            self._stamps.update(json.loads(response.stamps_json))
            return json.loads(response.result_json)

    @contextmanager
    def keyboard_motion(self):
        # Snapshot before the transport's epoch check. A STOP between that
        # check and RPC construction must still invalidate the old command.
        with self._generation_lock:
            generation = self._keyboard_generation
        previous = getattr(self._angle_sample, 'motion_generation', None)
        self._angle_sample.motion_generation = generation
        try:
            yield
        finally:
            if previous is None:
                del self._angle_sample.motion_generation
            else:
                self._angle_sample.motion_generation = previous

    def priority_stop(self, jog_axis=None):
        if self.feature != 'keyboard':
            raise RuntimeError('Priority STOP is reserved for keyboard control')
        packet = self._rpc('keyboard_stop', () if jog_axis is None else (int(jog_axis),))
        with self._generation_lock:
            self._keyboard_generation = max(self._keyboard_generation, packet['generation'])
        return packet['result']

    def stop(self, *args, **kwargs):
        if self.feature == 'keyboard':
            return self.priority_stop()
        return self._rpc('stop', args, kwargs)

    def _heartbeat(self):
        if not self._claimed or self._closed:
            return
        if self._heartbeat_pending is None or self._heartbeat_pending.done():
            self._heartbeat_pending = self.client.call_async(self._request('heartbeat'))

    def _sample(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in ARM_JOINTS):
            return
        wall = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9
        angles = tuple(math.degrees(values[name]) for name in ARM_JOINTS)
        measured = time.monotonic() - max(0., time.time() - wall)
        with self._condition:
            if self._latest is not None and wall <= self._latest.wall_time:
                return
            sequence = 1 if self._latest is None else self._latest.sequence + 1
            sample = SimpleNamespace(angles=angles, monotonic=measured, wall_time=wall, sequence=sequence)
            self._latest = sample
            self._stamps['arm'] = [measured, wall]
            self._condition.notify_all()
            callback = self._callback
        if callback:
            try:
                callback(sample)
            except Exception:
                pass  # The normal get_angles path validates and reports the same data.

    def get_angles(self):
        if self.feature == 'keyboard':
            # Pose save/arrival checks consume the public 20 Hz angle cache,
            # rather than adding SDK queries beside Cartesian Z monitoring.
            with self._condition:
                sample = self._latest
                if sample is None or time.monotonic() - sample.monotonic > KEYBOARD_ARM_FEEDBACK_MAX_AGE_S:
                    raise PublicArmFeedbackUnavailable('Public six-axis feedback is missing or older than 1.0 s')
                self._angle_sample.stamp = [sample.monotonic, sample.wall_time]
                self._stamps['arm'] = self._angle_sample.stamp
                return list(sample.angles)
        packet = self._rpc('get_angles', [self._rpc_sequence])
        self._rpc_sequence = packet['sequence']
        if packet['error']:
            raise RuntimeError(packet['error'])
        return packet['value']

    def sample_time(self, kind):
        if self.feature == 'keyboard' and kind == 'arm':
            return getattr(self._angle_sample, 'stamp', None)
        return self._stamps.get(kind)

    def set_angle_callback(self, callback):
        with self._condition:
            self._callback = callback

    def __getattr__(self, name):
        from pro630_broker_support import READS, WRITES
        if name not in READS | set().union(*WRITES.values()):
            raise AttributeError(name)
        return lambda *args, **kwargs: self._rpc(name, args, kwargs)

    def close(self):
        if self._closed:
            return
        try:
            if self._claimed:
                self._rpc('release')
        except Exception as exc:
            self.node.get_logger().warning(f'Feature lease release incomplete: {exc}')
        finally:
            self._closed = True
            self._claimed = False
            with self._condition:
                self._condition.notify_all()
            self.executor.shutdown(timeout_sec=1.)
            self.thread.join(timeout=1.)
            self.node.destroy_node()
