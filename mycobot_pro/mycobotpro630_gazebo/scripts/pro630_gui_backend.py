#!/usr/bin/env python3
"""Real-only GUI worker: SDK commands and SDK-reported completion, via the public owner."""
import json
import math
import queue
import signal
import threading
import time

import pro630_ros1 as ros
from pro630_ros1 import Node
from std_msgs.msg import Empty, String
from moveit_msgs.srv import GetStateValidity

from pro630_model import ARM_JOINT_SDK_SPEED_MAX, ARM_CARTESIAN_SDK_SPEED_MAX
from pro630_control_logging import ControlLog
from pro630_configuration import configure_node
from pro630_public_client import PublicPro630Client
from pro630_collision_check import CollisionCheck, DEFAULT_COLLISION_DEPTH_TOLERANCE_M
from pro630_real_keyboard import RealPoseReader


CARTESIAN_NOT_REACHED_STATUS = (
    'The current pose may be singular. '
    'Please use joint control to move away.')


class CommandStopped(Exception):
    pass


class CommandRejected(Exception):
    pass


class GuiBackend(CollisionCheck, Node):
    def __init__(self):
        super().__init__('pro630_gui_backend')
        configure_node(self)
        self._lock = threading.RLock()
        self._closing = threading.Event()
        self._stop = threading.Event()
        self._queue = queue.Queue(maxsize=1)
        self._busy = False
        self._active_kind = None
        self._gripper_active = False
        self._command_id = ''
        self._command_kind = None
        self.log = ControlLog(self.get_logger())
        self._status = 'Ready.'
        self._angles = self._coords = None
        self._feedback_when = 0.
        self._feedback_error = ''
        self._feedback_failure_since = None
        self.mc = None
        self.worker = None
        # This proxy claims the already-running real public bridge; it never
        # constructs an SDK connection or powers/enables any motor.
        self.mc = PublicPro630Client(feature='gui', with_gripper=self.with_gripper, allow_collision=self.allow_collision)
        self.validity_client = self.create_client(
            GetStateValidity, '/check_state_validity', persistent=True)
        self.collision_depth_tolerance_m = DEFAULT_COLLISION_DEPTH_TOLERANCE_M
        self._joint_pose_reader = RealPoseReader(
            self.mc, self.joint_limits, with_gripper=self.with_gripper,
            report=self.get_logger().debug)
        self.feedback_pub = self.create_publisher(String, '/pro630/gui/feedback', 1)
        self.create_subscription(String, '/pro630/gui/command', self.command, 10)
        self.create_subscription(Empty, '/pro630/gui_stop', self.stop, 10)
        self.create_timer(.05, self.feedback)
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()

    def status(self, text, level='info', *, context=None):
        with self._lock:
            self._status = text
            kind, command_id = self._command_kind, self._command_id
        if context is not None:
            kind, command_id = context
        self.log.event(f'[kind={kind}, id={command_id}] {text}', level)

    def feedback(self):
        with self._lock:
            data = dict(angles=self._angles, coords=self._coords,
                        age=min(1e6, time.monotonic()-self._feedback_when),
                        busy=self._busy, ready=True, status=self._status,
                        feedback_error=self._feedback_error, command_id=self._command_id)
        self.feedback_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    @staticmethod
    def six_values(values, label):
        if (not isinstance(values, (list, tuple)) or len(values) != 6
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise RuntimeError(f'Invalid SDK {label} reply: {values!r}')
        return list(values)

    def refresh_feedback(self):
        # Readback failures are displayed, not turned into a model/feedback gate
        # on the next command. SDK command replies remain authoritative.
        try:
            angles = self.six_values(self.mc.get_angles(), 'angles')
            coords = self.six_values(self.mc.get_coords(), 'coordinates')
            with self._lock:
                self._angles, self._coords = angles, coords
                self._feedback_when, self._feedback_error = time.monotonic(), ''
                self._feedback_failure_since = None
        except Exception:
            with self._lock:
                # A transient missing/invalid SDK reply is not a robot error.
                # Keep the last valid data and its age without flashing raw -1.
                now = time.monotonic()
                if self._feedback_failure_since is None:
                    self._feedback_failure_since = now
                if now - self._feedback_failure_since >= 1.:
                    self._feedback_error = 'SDK feedback unavailable; waiting for a valid response.'

    def command(self, msg):
        data = None
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or set(data) != {'id', 'kind', 'target', 'speed'}:
                raise ValueError('Invalid command fields')
            if not isinstance(data['id'], str) or not 1 <= len(data['id']) <= 64:
                raise ValueError('Invalid command identifier')
            if data['kind'] not in ('joints', 'coords', 'gripper'):
                raise ValueError('Unsupported command')
            if type(data['speed']) is not int or not 1 <= data['speed'] <= 100:
                raise ValueError('Speed must be an integer from 1 to 100')
            count = 1 if data['kind'] == 'gripper' else 6
            if (not isinstance(data['target'], list) or len(data['target']) != count
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in data['target'])):
                raise ValueError('Invalid target values')
            if data['kind'] == 'gripper' and not self.with_gripper:
                raise ValueError('No gripper is installed')
            if data['kind'] == 'gripper' and not 0 <= data['target'][0] <= 1:
                raise ValueError('Invalid gripper target')
            with self._lock:
                if self._busy or self._stop.is_set():
                    raise RuntimeError('A command or STOP is in progress')
                self._busy, self._command_id = True, data['id']
                self._command_kind = data['kind']
                self.status('Sending command...')
                self._queue.put_nowait(data)
        except Exception as exc:
            with self._lock:
                if not self._busy:
                    self._command_id = data.get('id', '') if isinstance(data, dict) else ''
            context = (data.get('kind'), data.get('id')) if isinstance(data, dict) else (None, None)
            self.status(f'Rejected: {exc}', 'warning', context=context)

    def stop(self, _msg):
        self._stop.set()

    def cancelled(self):
        if self._stop.is_set() or self._closing.is_set():
            raise CommandStopped()

    @property
    def _stop_requested(self):
        # Shared collision queries use the existing GUI cancellation events.
        return self._stop.is_set() or self._closing.is_set()

    def _joint_preflight_pose(self):
        self.cancelled()
        try:
            if self.with_gripper:
                self._joint_pose_reader.force_gripper_read()
            pose = self._joint_pose_reader(moving=False)
        except Exception as exc:
            self.cancelled()
            self.get_logger().debug(f'Joint collision preflight feedback: {exc}')
            raise CommandRejected('Current real robot pose is unavailable or stale.') from exc
        self.cancelled()
        return pose

    def _validate_joint_target(self, target):
        arm = [math.radians(value) for value in target]
        if any(not lo <= value <= hi for value, (lo, hi) in zip(arm, self.joint_limits[:6])):
            raise CommandRejected('Joint target exceeds its limits.')
        current = self._joint_preflight_pose()
        # Reuse the slider's full path checks, including its collision-exit rules.
        # Never move the gripper as a side effect of a six-axis GUI command.
        for _ in range(3):
            self.status('Validating joint path for collisions...')
            goal = arm + ([current[6]] if self.with_gripper else [])
            try:
                valid, reason = self._path_is_valid(current, goal)
            except Exception as exc:
                self.cancelled()
                raise CommandRejected(f'Collision validation failed: {exc}') from exc
            self.cancelled()
            if not valid:
                raise CommandRejected(reason)
            confirmed = self._joint_preflight_pose()
            changed = max(abs(a-b) for a, b in zip(current[:6], confirmed[:6])) > 0.01
            if self.with_gripper:
                changed = changed or abs(current[6] - confirmed[6]) > 1e-6
            if not changed:
                return
            current = confirmed
            self.status('Start pose changed; revalidating before motion...')
        raise CommandRejected('Start pose keeps changing; release external control and retry.')

    @staticmethod
    def motion_reply(result, *, asynchronous_send=False):
        accepted = (None, 0, 1)  # ElephantRobot writes return None; measured arrival confirms completion.
        if result is not None and (type(result) is not int or result not in accepted):
            raise RuntimeError('Motion rejected: controller rejected the command.')

    @staticmethod
    def gripper_reply(result):
        # Pro630 force commands return 1 on success; never mask a rejected write.
        if type(result) is not int or result != 1:
            raise RuntimeError('Command failed: gripper command rejected.')

    @staticmethod
    def controller_error(error):
        # Pro630 returns text and state bits, never Pro450 binary status codes.
        if isinstance(error, str) and error.startswith(('Motion rejected:', 'Motion failed:', 'Motion stopped')):
            return error
        return 'Motion failed: controller reported an error.'

    def arm_error(self, *, joint_motion=False):
        result = self.mc.get_error_information()
        if type(result) is int and result == 0:
            return
        if not isinstance(result, str):
            raise RuntimeError('Command failed: no controller response.')
        raise RuntimeError(self.controller_error(result))

    def stop_robot(self):
        errors = []
        try:
            self.motion_reply(self.mc.stop(_async=True))
        except Exception as exc:
            errors.append(str(exc))
        if self.with_gripper and self._gripper_active:
            try:
                opening = self.mc.get_pro_gripper_angle()
                if type(opening) not in (int, float) or not math.isfinite(opening) or not 0 <= opening <= 100:
                    raise RuntimeError(f'Invalid SDK gripper angle: {opening!r}')
                # This gripper has no usable SDK stop: retain its measured opening.
                self.gripper_reply(self.mc.set_pro_gripper_angle(int(round(opening))))
                self._gripper_active = False
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError('; '.join(errors))

    def execute(self, command):
        self.cancelled()
        kind, target, speed = command['kind'], command['target'], command['speed']
        if kind == 'coords' and target[2] < 0 and not self.allow_collision:
            raise CommandRejected('Target Z must be at least 0 mm.')
        if kind == 'joints':
            self._validate_joint_target(target)
            self.cancelled()
        self._active_kind = kind
        if kind == 'joints':
            self.motion_reply(self.mc.send_angles(target, max(1, round(speed * ARM_JOINT_SDK_SPEED_MAX / 100)), _async=True), asynchronous_send=True)
        elif kind == 'coords':
            self.motion_reply(self.mc.send_coords(target, max(1, round(speed * ARM_CARTESIAN_SDK_SPEED_MAX / 100)), _async=True), asynchronous_send=True)
        else:
            self.gripper_reply(self.mc.set_pro_gripper_speed(speed))
            self.cancelled()
            self._gripper_active = True
            self.gripper_reply(self.mc.set_pro_gripper_angle(int(round(target[0]*100.))))
        # Pro630 text writes acknowledge receipt, not completion of motion.
        self.status('Command sent; waiting for SDK target confirmation.')
        if self._stop.wait(.25):
            raise CommandStopped()
        deadline = time.monotonic()+600.
        idle_since = None
        reply = None
        while time.monotonic() < deadline:
            self.cancelled()
            if kind == 'gripper':
                result = self.mc.get_pro_gripper_status()
                if type(result) is not int or result not in (0, 1, 2, 3):
                    raise RuntimeError(f'SDK gripper status returned {result!r}')
                if result != 0:
                    self._gripper_active = False
                    description = {1: 'stopped', 2: 'stopped; object detected', 3: 'stopped; object lost'}[result]
                    self.status('SDK reports gripper ' + description + '.')
                    return
            else:
                reply = self.mc.get_motion_status()
                if reply is not None and reply['error'] is not None:
                    raise RuntimeError(self.controller_error(reply['error']))
                result = self.mc.is_moving()
                if type(result) is not int or result not in (0, 1):
                    raise RuntimeError('Command failed: no controller response.')
                if result == 0:
                    self.arm_error(joint_motion=kind == 'joints')
                    arrived = self.mc.is_in_position(target, 1 if kind == 'coords' else 0)
                    if type(arrived) is not int or arrived not in (0, 1):
                        raise RuntimeError('Command failed: no controller response.')
                    # A rejection may arrive while the status queries are waiting.
                    reply = self.mc.get_motion_status()
                    if reply is not None and reply['error'] is not None:
                        raise RuntimeError(self.controller_error(reply['error']))
                    if arrived == 1:
                        self.status('Motion complete: SDK confirmed the target position.')
                        return
                    # Allow controller startup/status latency without treating
                    # an idle robot as success or resending the motion command.
                    if idle_since is None:
                        idle_since = time.monotonic()
                    elif time.monotonic() - idle_since >= 3.:
                        if reply and reply['received']:
                            raise RuntimeError(CARTESIAN_NOT_REACHED_STATUS)
                        raise RuntimeError('Command failed: no controller response.')
                else:
                    idle_since = None
            self.refresh_feedback()
            self._stop.wait(.2)
        raise RuntimeError('SDK did not report completion within 600 seconds')

    def work(self):
        while not self._closing.is_set() and ros.ok():
            try:
                command = self._queue.get(timeout=.2)
            except queue.Empty:
                if self._closing.is_set():
                    break
                if self._stop.is_set():
                    try:
                        self.stop_robot()
                        self.status('STOP sent.')
                    except Exception as exc:
                        self.status(f'STOP failed: {exc}', 'error')
                    finally:
                        self._stop.clear()
                if not self._closing.is_set():
                    self.refresh_feedback()
                continue
            try:
                self.execute(command)
            except CommandRejected as exc:
                # Validation has not sent any motion: reject without SDK writes.
                self.status(f'Rejected: {exc}', 'warning')
            except CommandStopped:
                try:
                    self.stop_robot()
                    self.status('STOP sent.')
                except Exception as exc:
                    self.status(f'STOP failed: {exc}', 'error')
            except Exception as exc:
                message = str(exc)
                if message != CARTESIAN_NOT_REACHED_STATUS and not message.startswith(('Motion rejected:', 'Motion failed:', 'Motion stopped',
                                           'Motion not completed:', 'Command failed:')):
                    message = f'Command failed: {message}'
                if self._active_kind is not None:
                    try:
                        self.stop_robot()
                    except Exception as stop_error:
                        if message == CARTESIAN_NOT_REACHED_STATUS:
                            self.get_logger().error(f'STOP failed: {stop_error}')
                        else:
                            message += f'; STOP failed: {stop_error}'
                self.status(message, "error")
            finally:
                with self._lock:
                    self._active_kind = None
                    self._busy = False
                    self._stop.clear()
            if not self._closing.is_set():
                self.refresh_feedback()

    def close(self):
        self._closing.set()
        self._stop.set()
        if self.worker is not None:
            self.worker.join(timeout=2.)
        if self.mc is not None:
            # The public owner stops unfinished writes if this worker was lost.
            self.mc.close()


def main():
    from pro630_ros1 import SignalHandlerOptions
    ros.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        node = GuiBackend()
        ros.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.close()
            node.destroy_node()
        ros.try_shutdown()


if __name__ == '__main__':
    main()
