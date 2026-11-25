#!/usr/bin/env python3
"""Public real-pose owner: one SDK connection, serial calls and Gazebo mirror."""
import json
import math
import signal
import threading
import time
from contextlib import nullcontext

import pro630_ros1 as ros
from pro630_ros1 import ReentrantCallbackGroup
from pro630_ros1 import MultiThreadedExecutor
from pro630_ros1 import Node
from pro630_ros1 import get_logger
from pro630_ros1 import DurabilityPolicy, QoSProfile
from rospy import Duration, Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mycobotpro630_gazebo.srv import Pro630Rpc

from pro630_sdk_adapter import PositionLimitExceeded, Pro630Client
from pro630_session_support import LIMITS_DEG, read_initial_snapshot
from pro630_broker_support import FeatureLease, KEYBOARD_MOTION_METHODS
from pro630_real_mirror import RealPoseBuffer
from pro630_configuration import configure_node
from pro630_control_logging import (
    ControlLog, POSITION_LIMIT_MESSAGE, POSITION_LIMIT_SHUTDOWN_S, POSITION_LIMIT_TOPIC, emit_log,
)
from pro630_tool_reference import ToolReferenceContext
from pro630_model import DEFAULT_PRO630_IP, DEFAULT_PRO630_PORT


GRIPPER_SAMPLE_PERIOD = .1
GRIPPER_RETRY_PERIOD = .5
GRIPPER_MAX_SAMPLE_AGE = .8


class RobotBridge(Node):
    def __init__(self):
        super().__init__('pro630_robot_bridge')
        configure_node(self)
        self.sdk = None
        self.tool_reference = None
        self.hardware_lock = threading.RLock()
        self.keyboard_stop_lock = threading.Lock()
        self.cache_condition = threading.Condition()
        self.stop_event = threading.Event()
        self.lease = FeatureLease()
        self.stamps = {}
        self.packet = {'sequence': 0, 'value': None, 'error': 'No arm feedback'}
        self.arm_active = self.gripper_active = False
        self.arm_goal = None
        self.mirror = RealPoseBuffer(pose_size=len(self.command_joints))
        self.gripper_mirror = (RealPoseBuffer(pose_size=1,
                              max_sample_age=GRIPPER_MAX_SAMPLE_AGE)
                               if self.with_gripper else None)
        self.next_gripper = 0.
        self.worker = None
        self.frame = None
        self.environment_ready = False
        self.environment_seen = 0.
        self._feedback_error_log_times = {}
        self.position_limit_deadline = None
        self.transport_log = ControlLog(self.get_logger())
        try:
            self.declare_parameter('pro630_ip', DEFAULT_PRO630_IP)
            self.declare_parameter('pro630_port', DEFAULT_PRO630_PORT)
            self.sdk = Pro630Client(self.get_parameter('pro630_ip').value,
                                   int(self.get_parameter('pro630_port').value),
                                   feedback_hz=20., ownership_timeout=0)
            self.tool_reference = ToolReferenceContext(self.sdk)
            self.initial_pose = read_initial_snapshot(self.sdk, with_gripper=self.with_gripper)
            self.gripper = self.initial_pose[6] if self.with_gripper else None
            self.stamps = {kind: self.sdk.sample_time(kind) for kind in (('arm', 'gripper') if self.with_gripper else ('arm',))}
            if self.with_gripper and self.stamps['gripper'] is not None:
                self.gripper_mirror.push([self.gripper], when=self.stamps['gripper'][0])
            self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/command', 1)
            self.gripper_pub = (self.create_publisher(JointTrajectory, '/pro_gripper_controller/command', 1)
                                if self.with_gripper else None)
            self.real_pub = self.create_publisher(JointState, '/pro630/real_joint_states', 1)
            self.age_pub = self.create_publisher(Float64MultiArray, '/pro630/real_feedback_age', 1)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.snapshot_pub = self.create_publisher(JointState, '/pro630/real_state_snapshot', qos)
            self.position_limit_pub = self.create_publisher(Bool, POSITION_LIMIT_TOPIC, qos)
            self.create_subscription(String, '/pro630/environment_state', self.environment_state, qos)
            self.create_service(Pro630Rpc, '/pro630/robot_rpc', self.rpc,
                                callback_group=ReentrantCallbackGroup())
            self.create_timer(.02, self.render)
            self.create_timer(.2, self.publish_snapshot)
            self.create_timer(.2, self.check_health)
            self.worker = threading.Thread(target=self.sample_loop, daemon=True)
            self.worker.start()
            self.get_logger().info('Public robot connection acquired; continuous tracking active. No startup motion sent.')
        except Exception:
            self.close()
            raise

    def log_transport_error(self, reason):
        # Sampling and the executor can observe the same closed socket together.
        self.transport_log.state('transport', reason, f'Robot connection lost: {reason}', 'error')

    def check_health(self):
        if self.sdk is not None and self.sdk._closed.is_set():
            reason = self.sdk.get_transport_error() or 'Pro630 transport closed'
            self.log_transport_error(reason)
            raise RuntimeError(f'{reason}; restart the public environment after checking the connection')
        deadline = self.position_limit_deadline
        if deadline is not None and time.monotonic() >= deadline:
            raise PositionLimitExceeded(POSITION_LIMIT_MESSAGE)

    def log_feedback_error(self, kind, exc):
        now = time.monotonic()
        reason = self.sdk.get_transport_error()
        if reason:
            self.log_transport_error(reason)
            return
        if now - self._feedback_error_log_times.get(kind, float('-inf')) >= 5.:
            self._feedback_error_log_times[kind] = now
            emit_log(self.get_logger(), f'{kind} feedback failed: {exc}', 'warning')

    def environment_state(self, msg):
        try:
            state = json.loads(msg.data)
            if state.get('environment') == 'real':
                self.environment_ready = state.get('ready') is True
                self.environment_seen = time.monotonic()
        except (ValueError, TypeError):
            pass

    def update_gripper(self, value):
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and 0 <= value <= 100:
            stamp = self.sdk.sample_time('gripper')
            previous = self.stamps.get('gripper')
            if stamp is not None and (previous is None or stamp[0] > previous[0]):
                self.gripper = value / 100.
                self.stamps['gripper'] = stamp
                self.gripper_mirror.push([self.gripper], when=stamp[0])
                # A feature RPC may supply a sample between periodic reads.
                self.next_gripper = max(self.next_gripper, stamp[0] + GRIPPER_SAMPLE_PERIOD)
                return True
        return False

    def report_position_limit(self):
        if self.position_limit_deadline is not None:
            return
        self.position_limit_deadline = time.monotonic() + POSITION_LIMIT_SHUTDOWN_S
        self.transport_log.event(POSITION_LIMIT_MESSAGE, 'error')
        self.position_limit_pub.publish(Bool(data=True))

    def sample_loop(self):
        while not self.stop_event.is_set() and not self.sdk._closed.is_set():
            if self.lease.expired():
                try:
                    with self.hardware_lock:
                        if self.lease.expired():
                            self.release_owner(self.lease.token)
                except Exception as exc:
                    self.transport_log.state('lease_release', (self.lease.token, str(exc)),
                        f'Expired feature stop failed: {exc}', 'error')
            value, error = None, ''
            try:
                with self.hardware_lock:
                    value = self.sdk.get_angles()
                    stamp = self.sdk.sample_time('arm')
                    self.stamps['arm'] = stamp
                    valid = (isinstance(value, (list, tuple)) and len(value) == 6
                             and all(not isinstance(v, bool) and isinstance(v, (int, float))
                                     and math.isfinite(v) and lo <= v <= hi
                                     for v, (lo, hi) in zip(value, LIMITS_DEG)))
                    if valid and stamp is not None:
                        self.stamps['arm'] = stamp
                        pose = [math.radians(v) for v in value] + ([self.gripper] if self.with_gripper else [])
                        previous = self.frame
                        velocity = [0.] * len(self.command_joints) if previous is None else [
                            (current - old) / max(.001, stamp[0] - previous[0])
                            for current, old in zip(pose, previous[2])]
                        self.frame = (stamp[0], stamp[1], pose, velocity)
                        self.mirror.push(pose, when=stamp[0])
                        if self.arm_goal and max(abs(a-b) for a,b in zip(value, self.arm_goal)) < .5:
                            self.arm_active = False
            except PositionLimitExceeded as exc:
                error = str(exc)
                self.report_position_limit()
            except Exception as exc:
                error = str(exc)
                self.log_feedback_error('Arm', exc)
            with self.cache_condition:
                self.packet = {'sequence': self.packet['sequence'] + 1, 'value': value, 'error': error}
                self.cache_condition.notify_all()
            if self.sdk._closed.is_set():
                return
            # Publish the arm packet before a possibly slow gripper query.
            if self.with_gripper and time.monotonic() >= self.next_gripper:
                with self.hardware_lock:
                    if not self.stop_event.is_set() and time.monotonic() >= self.next_gripper:
                        started = time.monotonic()
                        updated = False
                        try:
                            updated = self.update_gripper(self.sdk.get_pro_gripper_angle())
                        except Exception as exc:
                            # Preserve the last measured opening and timestamp.
                            self.log_feedback_error('Gripper', exc)
                        # Schedule from query start; do not add another full
                        # sample period after each successful SDK response.
                        self.next_gripper = (max(started + GRIPPER_SAMPLE_PERIOD,
                                                 time.monotonic() + .005) if updated
                                             else time.monotonic() + GRIPPER_RETRY_PERIOD)
            self.stop_event.wait(.005)

    def release_owner(self, token):
        # Called under the hardware lock. Never change motor enable on feature exit.
        self.lease.touch(token)
        if self.arm_active:
            self.sdk.stop(deceleration=0, _async=True)
            self.arm_active = False
        if self.with_gripper and self.gripper_active:
            opening = self.sdk.get_pro_gripper_angle()
            if not isinstance(opening, (int, float)) or isinstance(opening, bool) or not 0 <= opening <= 100:
                raise RuntimeError('Cannot hold gripper: fresh opening unavailable')
            self.sdk.set_pro_gripper_angle(int(round(opening)))
            self.gripper_active = False
            self.update_gripper(opening)
        if self.lease.feature in ('gui', 'keyboard') and self.tool_reference is not None:
            self.tool_reference.restore()
        self.lease.clear(token)
        self.transport_log.clear('lease_release')

    def keyboard_stop(self, token, jog_axis=None):
        # Invalidate queued motions immediately. Native STOP still serializes
        # after the in-flight text reply; never interleave an unframed stream.
        with self.keyboard_stop_lock, self.lease.lock:
            self.lease.authorize(token, 'keyboard_stop')
            barrier = self.sdk.keyboard_stop_barrier
            generation = barrier.begin_stop()
            if jog_axis is None:
                result = self.sdk.stop(deceleration=0, _async=True)
            else:
                result = self.sdk.jog_stop(jog_axis)
            if result is not None and (type(result) is not int or result not in (0, 1)):
                raise RuntimeError(f'Robot STOP failed: {result!r}')
            self.arm_active, self.arm_goal = False, None
            self.lease.touch(token)
            barrier.finish_stop(generation)
            return {'generation': generation, 'result': result}

    def rpc(self, req, res):
        try:
            args, kwargs = json.loads(req.args_json or '[]'), json.loads(req.kwargs_json or '{}')
            if not isinstance(args, list) or not isinstance(kwargs, dict):
                raise ValueError('Invalid request payload')
            method = req.method
            if not self.with_gripper and 'gripper' in method:
                raise RuntimeError('Gripper operations are unavailable in the arm-only environment')
            if method == 'get_capabilities':
                result = {'with_gripper': self.with_gripper, 'joints': self.command_joints,
                          'allow_collision': self.allow_collision}
            elif method == 'heartbeat':
                self.lease.touch(req.token)
                result = True
            elif method == 'claim':
                with self.hardware_lock:
                    if not self.environment_ready or time.monotonic() - self.environment_seen > 2.:
                        raise RuntimeError('Public environment is not ready for a control feature')
                    # Reconcile any failed setup rollback before another feature
                    # can acquire the SDK-local tool coordinate context.
                    if self.lease.token is None and self.tool_reference is not None:
                        self.tool_reference.restore()
                    already_owned = self.lease.token == req.token
                    if already_owned and self.lease.feature != args[0]:
                        raise RuntimeError('Control feature does not match the existing owner')
                    self.lease.claim(req.token, args[0])
                    if not already_owned and args[0] in ('keyboard', 'gui'):
                        try:
                            self.tool_reference.apply(with_gripper=self.with_gripper)
                            self.lease.touch(req.token)
                            self.get_logger().info(
                                f'{args[0]} coordinate settings ready; ' +
                                ('SDK tool Z offset=165 mm.' if self.with_gripper else 'flange control.'))
                        except Exception:
                            self.lease.clear(req.token)
                            raise
                    self.mirror.clear()
                result = ({'generation': self.sdk.keyboard_stop_barrier.generation}
                          if args[0] == 'keyboard' else True)
            elif method == 'keyboard_stop' or (method == 'stop' and self.lease.feature == 'keyboard'):
                if method == 'stop' and (args or kwargs != {'deceleration': 0, '_async': True}):
                    raise ValueError('Invalid keyboard STOP arguments')
                if method == 'keyboard_stop' and (kwargs or len(args) > 1 or (
                        args and (type(args[0]) is not int or not 0 <= args[0] < 6))):
                    raise ValueError('Invalid keyboard STOP arguments')
                result = self.keyboard_stop(req.token, args[0] if method == 'keyboard_stop' and args else None)
                if method == 'stop':
                    result = result['result']
            elif method == 'get_angles':
                self.lease.authorize(req.token, method)
                deadline = time.monotonic() + .6
                with self.cache_condition:
                    while (self.packet['sequence'] == 0 or self.packet['sequence'] <= args[0]) and time.monotonic() < deadline:
                        self.cache_condition.wait(max(0., deadline - time.monotonic()))
                    if self.packet['sequence'] == 0 or self.packet['sequence'] <= args[0]:
                        raise RuntimeError('Public arm sampling timed out')
                    result = dict(self.packet)
            else:
                with self.hardware_lock:
                    if method == 'release':
                        self.release_owner(req.token)
                        result = True
                    else:
                        self.lease.authorize(req.token, method)
                        if method == 'set_motor_enabled' and (args not in ([254, 0], [254, 1]) or kwargs):
                            raise ValueError('Drag mode may switch only all six arm motors')
                        if method == 'set_pro_gripper_enabled' and (
                                len(args) != 1 or type(args[0]) is not int or args[0] not in (0, 1) or kwargs):
                            raise ValueError('Drag mode may only enable or disable the default gripper')
                        keyboard_motion = self.lease.feature == 'keyboard' and method in KEYBOARD_MOTION_METHODS
                        generation = kwargs.pop('_keyboard_generation', None) if keyboard_motion else None
                        barrier = self.sdk.keyboard_stop_barrier
                        context = barrier.command(generation) if keyboard_motion else nullcontext()
                        with context:
                            with barrier.guard(generation) if keyboard_motion else nullcontext():
                                if self.lease.feature in ('gui', 'keyboard'):
                                    if method in ('send_angles', 'send_coords', 'jog_coord'):
                                        self.arm_active, self.arm_goal = True, None
                                    elif method == 'set_pro_gripper_angle':
                                        self.gripper_active = True
                            result = getattr(self.sdk, method)(*args, **kwargs)
                            with barrier.guard(generation) if keyboard_motion else nullcontext():
                                if self.lease.feature == 'gui':
                                    if method == 'is_in_position' and result == 1:
                                        self.arm_active, self.arm_goal = False, None
                                    elif method == 'get_pro_gripper_status' and result in (1, 2, 3):
                                        self.gripper_active = False
                                if method == 'get_pro_gripper_angle':
                                    self.update_gripper(result)
                                elif method == 'set_pro_gripper_angle' and result == 1:
                                    self.gripper_active = True
                                elif method in ('send_angles', 'jog_coord'):
                                    self.arm_active = True
                                    self.arm_goal = list(args[0]) if method == 'send_angles' else None
                                elif method == 'stop':
                                    if result is not None and result not in (0, 1):
                                        raise RuntimeError(f'Robot STOP failed: {result!r}')
                                    self.arm_active = False
                                    self.arm_goal = None
            res.success = True
            res.result_json = json.dumps(result, allow_nan=False)
            res.stamps_json = json.dumps(self.stamps)
        except Exception as exc:
            res.success, res.error = False, str(exc)
            res.stamps_json = '{}'
        return res

    @staticmethod
    def trajectory(names, pose, velocity):
        msg = JointTrajectory()
        msg.joint_names = list(names)
        msg.points = [JointTrajectoryPoint(positions=list(pose), velocities=list(velocity),
                         time_from_start=Duration(nsecs=40000000))]
        return msg

    def render(self):
        now = time.monotonic()
        arm_stamp, grip_stamp = self.stamps.get('arm'), self.stamps.get('gripper')
        ages = Float64MultiArray()
        ages.data = [now - arm_stamp[0] if arm_stamp else -1., (now - grip_stamp[0] if grip_stamp else -1.) if self.with_gripper else float('inf')]
        self.age_pub.publish(ages)
        frame = self.frame
        if frame is None or now - frame[0] > .5:
            return
        measured, wall, pose, measured_velocity = frame
        msg = JointState()
        sec = int(wall)
        msg.header.stamp = Time(secs=sec, nsecs=int((wall-sec)*1e9))
        msg.name, msg.position = list(self.command_joints), list(pose[:6]) + ([self.gripper] if self.with_gripper else [])
        msg.velocity = list(measured_velocity)
        self.real_pub.publish(msg)
        if self.lease.feature == 'drag':
            shown, velocity = list(pose[:6]) + ([self.gripper] if self.with_gripper else []), [0.] * len(self.command_joints)
        else:
            rendered = self.mirror.render()
            if rendered is None:
                return
            shown, velocity = rendered
        # Interpolate the gripper's own measured history, never arm timestamps
        # or commanded goals. Zero terminal velocity prevents extrapolation
        # after feedback stops, including at an opening-direction reversal.
        if self.with_gripper:
            rendered_gripper = self.gripper_mirror.render(now=now)
            shown[6] = rendered_gripper[0][0] if rendered_gripper is not None else self.gripper
            velocity[6] = 0.0
        self.arm_pub.publish(self.trajectory(self.command_joints[:6], shown[:6], velocity[:6]))
        if self.with_gripper:
            self.gripper_pub.publish(self.trajectory(self.command_joints[6:], shown[6:], velocity[6:]))

    def publish_snapshot(self):
        if self.frame is None or time.monotonic() - self.frame[0] > .5:
            return
        # This is an initial-pose source, not an SDK ownership handoff.
        msg = JointState()
        wall = min(self.frame[1], self.stamps['gripper'][1]) if self.with_gripper else self.frame[1]
        sec = int(wall); msg.header.stamp = Time(secs=sec, nsecs=int((wall-sec)*1e9))
        msg.header.frame_id = 'pro630:retain:' + self.sdk.owner_token
        msg.name, msg.position = list(self.command_joints), list(self.frame[2][:6]) + ([self.gripper] if self.with_gripper else [])
        self.snapshot_pub.publish(msg)

    def close(self):
        clean = True
        self.stop_event.set()
        if self.worker is not None:
            try:
                self.worker.join(timeout=1.)
            except Exception as exc:
                clean = False
                self.get_logger().error(f'Shutdown sampler cleanup failed: {exc}')
        if self.sdk is not None:
            try:
                with self.hardware_lock:
                    if (self.lease.token is None and self.tool_reference is not None
                            and self.tool_reference.saved is not None):
                        try:
                            self.tool_reference.restore()
                        except Exception as exc:
                            clean = False
                            self.get_logger().error(f'Control tool reference restoration failed: {exc}')
                    if self.lease.token is not None:
                        if self.sdk._closed.is_set():
                            clean = False
                            self.get_logger().error(
                                'Shutdown: robot stop/hold could not be confirmed (connection closed).')
                            if self.tool_reference is not None and self.tool_reference.saved is not None:
                                self.tool_reference.restore()
                        else:
                            try:
                                self.release_owner(self.lease.token)
                            except Exception as exc:
                                clean = False
                                self.get_logger().error(f'Shutdown stop/hold failed: {exc}')
                        if self.lease.token is not None:
                            try:
                                self.lease.clear(self.lease.token)
                            except Exception as exc:
                                clean = False
                                self.get_logger().error(f'Shutdown feature lease cleanup failed: {exc}')
            finally:
                try:
                    self.sdk.close()
                except Exception as exc:
                    clean = False
                    self.get_logger().error(f'Shutdown connection cleanup failed: {exc}')
                finally:
                    self.sdk = None
        return clean


def main():
    ros.init()
    node = None
    executor = MultiThreadedExecutor(num_threads=6)
    code = 0
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        node = RobotBridge()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except PositionLimitExceeded as exc:
        if node is None:
            emit_log(get_logger('pro630_robot_bridge'),
                     f'Public Pro630 bridge failed: {exc} ({POSITION_LIMIT_MESSAGE})', 'error')
        code = 1
    except Exception as exc:
        reason = node.sdk.get_transport_error() if node is not None and node.sdk is not None else None
        if reason and str(exc).startswith(reason):
            node.log_transport_error(reason)
            message = 'Public robot bridge stopping: connection lost. Check the connection and restart slider.launch.'
        else:
            message = f'Public Pro630 bridge failed: {exc}'
        emit_log(node.get_logger() if node is not None else get_logger('pro630_robot_bridge'), message, 'error')
        code = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        executor.shutdown(timeout_sec=1.)
        if node is not None:
            try:
                if not node.close():
                    code = 1
            finally:
                node.destroy_node()
        ros.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
