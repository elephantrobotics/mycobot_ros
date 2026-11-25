"""Pro630 feature contract backed by the installed ElephantRobot text SDK.

One serialized TCP connection, receive timestamps and model-coordinate
conversion. The SDK is unmodified. Pro450 packets/speed calibration are not
used. A transport timeout closes the unframed stream to reject late replies.
"""
import hashlib
import math
import os
import socket
import select
import tempfile
import threading
import time
import uuid
from contextlib import contextmanager
from pro630_broker_support import KeyboardStopBarrier, KeyboardCommandCancelled
from pro630_model import (ARM_CARTESIAN_SDK_SPEED_MAX, ARM_JOINT_SDK_SPEED_MAX, DEFAULT_PRO630_IP,
                         DEFAULT_PRO630_PORT, GRIPPER_ID,
                         HARDWARE_OFFSETS_DEG, LIMITS_DEG)
try:
    from pymycobot import ElephantRobot as _VendorClient
except ImportError:
    _VendorClient = object


TCP_TIMEOUT_S = 2.0
GRIPPER_ANGLE_TIMEOUT_S = 3.0


class RobotConnectionLease:
    """Host-local ownership released by the OS when the descriptor closes."""
    def __init__(self, ip, port, timeout=15.):
        key = hashlib.sha256(f'{ip}:{port}'.encode()).hexdigest()[:20]
        self.handle = open(os.path.join(tempfile.gettempdir(), f'pro630-{key}.lock'), 'a+b')
        self.handle.seek(0, os.SEEK_END)
        if not self.handle.tell():
            self.handle.write(b'0'); self.handle.flush()
        deadline = time.monotonic() + timeout
        while True:
            try:
                if os.name == 'nt':
                    import msvcrt
                    self.handle.seek(0)
                    msvcrt.locking(self.handle.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    self.handle.close(); self.handle = None
                    raise RuntimeError(f'Pro630 connection already owned for {ip}:{port}')
                time.sleep(.05)

    def close(self):
        if self.handle is not None:
            self.handle.close(); self.handle = None


class PositionLimitExceeded(RuntimeError):
    """Measured arm feedback is outside the model's joint bounds."""


class ControllerRejected(RuntimeError):
    """A complete controller rejection is not a broken TCP stream."""


class TransportClosedError(RuntimeError):
    """The unframed connection cannot be reused after a transport failure."""



class PriorityApiLock:
    """Reentrant request/reply lock; pending STOP precedes new requests."""
    def __init__(self):
        self.lock = threading.RLock()
        self.pending = threading.Event()
        self.pending_lock = threading.Lock()
        self.pending_count = 0
        self.local = threading.local()

    def __enter__(self):
        while True:
            nested = getattr(self.local, 'depth', 0) > 0
            priority = getattr(self.local, 'priority', False)
            if not nested and not priority and self.pending.is_set():
                time.sleep(.005)
                continue
            self.lock.acquire()
            if nested or priority or not self.pending.is_set():
                self.local.depth = getattr(self.local, 'depth', 0) + 1
                return self
            self.lock.release()

    def __exit__(self, *unused):
        self.local.depth -= 1
        self.lock.release()

    @contextmanager
    def priority(self):
        with self.pending_lock:
            self.pending_count += 1
            self.pending.set()
        self.local.priority = True
        try:
            with self:
                yield
        finally:
            self.local.priority = False
            with self.pending_lock:
                self.pending_count -= 1
                if self.pending_count == 0:
                    self.pending.clear()


class Pro630Client(_VendorClient):
    """Only this module maps feature operations to Pro630 vendor methods."""
    def __init__(self, ip=DEFAULT_PRO630_IP, netport=DEFAULT_PRO630_PORT,
                 debug=False, feedback_hz=20., ownership_timeout=15.):
        if _VendorClient is object:
            raise RuntimeError('pymycobot and numpy are required for real Pro630 mode')
        self._api_lock = PriorityApiLock()
        self.keyboard_stop_barrier = KeyboardStopBarrier()
        self._tool_control = False
        self._closed = threading.Event()
        self._transport_error = None
        self._stamps = {}
        self._last_reply = ''
        self._motion_reply = None
        self._lease = None
        self._period = 1. / max(1., min(20., feedback_hz))
        self._next_angles = 0.
        self.owner_token = uuid.uuid4().hex
        self._lease = RobotConnectionLease(ip, netport, ownership_timeout)
        try:
            super().__init__(ip, netport, debug)
            self.tcp_client.settimeout(TCP_TIMEOUT_S)
            self.tcp_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            if not super().start_client():
                raise RuntimeError(f'Cannot connect to Pro630 at {ip}:{netport}')
        except Exception:
            self.close()
            raise

    def send_command(self, command):
        with self._api_lock:
            if self._closed.is_set():
                raise TransportClosedError(self.get_transport_error() or
                                           'Pro630 connection closed; restart the public environment')
            method = command.split('(', 1)[0]
            motion = method in ('set_angles', 'set_coords', 'jog_coord')
            if motion:
                self._motion_reply = {'received': False, 'error': None}
            data = bytearray()
            try:
                # Firmware can return an invalid opening (255) just after 2 s.
                # Receive that complete reply instead of racing its arrival and
                # discarding the stream. Every other command retains 2 s.
                timeout = (GRIPPER_ANGLE_TIMEOUT_S if method == 'force_GetGripperAngle'
                           else TCP_TIMEOUT_S)
                self.tcp_client.settimeout(timeout)
                with self.keyboard_stop_barrier.write():
                    self.tcp_client.sendall((command.rstrip('\n') + '\n').encode())
                while len(data) < 65536:
                    chunk = self.tcp_client.recv(4096)
                    if not chunk:
                        raise ConnectionError('Pro630 peer disconnected')
                    data.extend(chunk)
                    received = (time.monotonic(), time.time())
                    if hasattr(socket, 'TCP_QUICKACK'):
                        try:
                            self.tcp_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
                        except OSError:
                            pass  # Optional Linux latency hint; correctness does not depend on it.
                    text = data.decode()
                    if ':' in text:
                        reply = text.split(':', 1)[1].strip()
                        if method == 'get_robot_state' and len(reply) < 64 and '\n' not in text:
                            continue
                        if reply in ('-', '+'):
                            continue
                        if (method in ('force_GetGripperAngle', 'force_GetSpeed')
                                and '\n' not in text and select.select([self.tcp_client], [], [], .05)[0]):
                            continue
                        if reply and (not reply.startswith('[') or ']' in reply):
                            break
                else:
                    raise RuntimeError('Pro630 response exceeds 64 KiB')
                if ':' not in text:
                    raise RuntimeError(f'Malformed Pro630 response: {text!r}')
                reply = text.split(':', 1)[1].strip()
                if not reply or '\n' in reply or '\r' in reply:
                    raise RuntimeError(f'Invalid Pro630 response: {reply!r}')
                if motion:
                    self._motion_reply['received'] = True
                if (motion and (reply == '-1' or any(reason in reply.lower() for reason in (
                        'singular', 'no solution', 'no coordinate solution', 'no cartesian solution', 'no inverse',
                        'exceed', 'out of range', 'not enabled', 'disabled', 'collision')))) or any(
                        word in reply.lower() for word in ('error', 'failed', 'exception')):
                    message = self._describe_rejection(reply)
                    if motion:
                        self._motion_reply['error'] = message
                    raise ControllerRejected(message)
                self._last_reply = reply
                self._received = received
                return reply
            except (ControllerRejected, KeyboardCommandCancelled):
                raise
            except (OSError, UnicodeError, RuntimeError) as exc:
                # Preserve the first failure before later closed-connection reads.
                # Include only a bounded reply prefix, not motion arguments.
                if self._transport_error is None:
                    self._transport_error = (
                        f'Pro630 transport failed during {method}: '
                        f'{type(exc).__name__}: {exc}; reply_prefix={bytes(data[:256])!r}')
                try:
                    self.close()
                except Exception as cleanup_exc:
                    self._transport_error += (
                        f'; connection cleanup failed: {type(cleanup_exc).__name__}: {cleanup_exc}')
                raise TransportClosedError(self._transport_error) from exc

    def get_transport_error(self):
        """Return the original failure without querying hardware."""
        return self._transport_error

    @staticmethod
    def _describe_rejection(reply):
        text = reply.lower()
        if 'singular' in text:
            return 'Motion rejected: singular position.'
        if ('coordinate' in text or 'coord' in text) and ('limit' in text or 'out of range' in text):
            return 'Motion rejected: coordinates exceed limits.'
        if any(reason in text for reason in ('no solution', 'no inverse', 'no coordinate solution', 'no cartesian solution')):
            return 'Motion rejected: no Cartesian solution.'
        if 'disabled' in text or 'not enabled' in text:
            return 'Motion rejected: robot disabled.'
        if 'collision' in text:
            return 'Motion stopped: controller collision detected.'
        if reply == '-1':
            return 'Motion rejected: controller rejected the target.'
        # Preserve a controller's explicit reason, not a Pro450 numeric mapping.
        return 'Motion failed: ' + reply.rstrip('.') + '.'

    def get_motion_status(self):
        with self._api_lock:
            return None if self._motion_reply is None else dict(self._motion_reply)

    @staticmethod
    def _six_finite(values):
        if (not isinstance(values, (list, tuple)) or len(values) != 6 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise ValueError('Six finite values are required')
        return list(values)

    def get_coords(self):
        with self._api_lock:
            reply = self.send_command('get_coords()')
            raw = self._six_finite([float(v.strip()) for v in reply.strip('[]').split(',')])
            if self._tool_control:
                import numpy as np
                from pymycobot.elephantrobot import flangeToTool
                raw = flangeToTool(np.array(raw), self.tool_matrix).tolist()[:3] + raw[3:]
            return self._six_finite(raw)

    def send_coords(self, coords, speed, _async=False):
        with self._api_lock:
            target = self._six_finite(coords)
            if self._tool_control:
                import numpy as np
                from pymycobot.elephantrobot import toolToflange
                target = toolToflange(np.array(target), self.tool_matrix).tolist()[:3] + target[3:]
            return super().write_coords(target, self._speed(speed, ARM_CARTESIAN_SDK_SPEED_MAX))

    def jog_coord(self, axis, direction, speed, _async=False):
        if type(axis) is not int or axis not in (1, 2, 3, 4, 5, 6) or type(direction) is not int or direction not in (0, 1):
            raise ValueError('Coordinate axis must be 1..6 and direction must be 0 or 1')
        return super().jog_coord(('X', 'Y', 'Z', 'RX', 'RY', 'RZ')[axis-1], 1 if direction else -1,
                                 self._speed(speed, ARM_CARTESIAN_SDK_SPEED_MAX))

    def is_in_position(self, target, mode=0):
        target = self._six_finite(target)
        if mode == 1:
            return int(super().coords_equal(target, self.get_coords()))
        if mode == 0:
            return int(super().angles_equal(target, self.get_angles()))
        raise ValueError('Position mode must be joint (0) or Cartesian (1)')

    def get_pro_gripper_status(self):
        status = super().force_get_gripper(GRIPPER_ID)
        if type(status) is not int or status not in (0, 1, 2, 3):
            raise RuntimeError('Invalid Pro630 gripper status')
        return status

    def sample_time(self, kind):
        return self._stamps.get(kind)

    def _flag(self, command):
        reply = self.send_command(command)
        if reply not in ('0', '1'):
            raise RuntimeError(f'Invalid Pro630 status for {command}: {reply!r}')
        return int(reply)

    def is_power_on(self):
        return self._flag('is_power_on()')

    def is_moving(self):
        return self._flag('check_running()')

    def get_error_information(self):
        state = super().get_robot_state().strip()
        if len(state) != 64 or set(state) - {'0', '1'}:
            raise RuntimeError(f'Invalid Pro630 robot state: {state!r}')
        # The SDK's bit-1 description contradicts its normal-state example.
        # On this Pro630 it stays zero both before and after an E-stop.
        # Do not label it as E-stop; retain the documented power/fault checks.
        bad = [i for i in (0, *range(2, 8), *range(24, 30)) if state[i] != '1']
        if state[30] == '1':
            bad.append(30)
        if not bad:
            return 0
        if state[0] != '1':
            return 'Motion stopped: robot power is off.'
        if state[30] == '1':
            return 'Motion stopped: controller collision detected.'
        for index in range(2, 8):
            if state[index] != '1':
                return f'Motion failed: joint {8-index} motor fault.'
        for index in range(24, 30):
            if state[index] != '1':
                return f'Motion failed: joint {30-index} communication error.'
        return 'Motion failed: controller reported an error.'

    def get_angles(self):
        with self._api_lock:
            delay = self._next_angles - time.monotonic()
            if delay > 0 and self._closed.wait(delay):
                raise RuntimeError('Pro630 connection closed')
            self._next_angles = time.monotonic() + self._period
            reply = self.send_command('get_angles()')
            # Vendor string_to_coords returns a plausible six-angle sentinel
            # on parse failure. Strict parsing prevents it becoming feedback.
            raw = [float(v.strip()) for v in reply.strip('[]').split(',')]
            if len(raw) != 6 or not all(math.isfinite(v) for v in raw):
                raise RuntimeError(f'Invalid Pro630 angle response: {reply!r}')
            values = [v + offset for v, offset in zip(raw, HARDWARE_OFFSETS_DEG)]
            if any(not lo <= v <= hi for v, (lo, hi) in zip(values, LIMITS_DEG)):
                raise PositionLimitExceeded('Pro630 arm feedback violates model joint limits')
            self._stamps['arm'] = self._received
            return values

    @staticmethod
    def _speed(value, maximum):
        if type(value) not in (int, float) or not math.isfinite(value) or not 1 <= value <= maximum:
            raise ValueError(f'Pro630 SDK speed must be in 1..{maximum}')
        return int(value)

    def send_angles(self, angles, speed, _async=False):
        with self._api_lock:
            if not isinstance(angles, (list, tuple)) or len(angles) != 6:
                raise ValueError('Six model-frame angles are required')
            if any(type(v) not in (int, float) or not math.isfinite(v) or not lo <= v <= hi
                   for v, (lo, hi) in zip(angles, LIMITS_DEG)):
                raise ValueError('Pro630 command violates model joint limits')
            raw = [v - offset for v, offset in zip(angles, HARDWARE_OFFSETS_DEG)]
            # Vendor write_angles returns None. Arrival is checked from
            # measured feedback rather than inventing a motion acknowledgment.
            return super().write_angles(raw, self._speed(speed, ARM_JOINT_SDK_SPEED_MAX))

    def stop(self, deceleration=0, _async=False):
        with self._api_lock.priority():
            return super().task_stop()  # Consume the reply before another request.

    def jog_stop(self, axis):
        # Socket jog_coord direction 0 stops that axis. task_stop only stops a program.
        if type(axis) is not int or not 0 <= axis < 6:
            raise ValueError('Jog stop axis must be 0..5')
        name = ('X', 'Y', 'Z', 'RX', 'RY', 'RZ')[axis]
        with self._api_lock.priority():
            reply = self.send_command(f'jog_coord({name},0,1)')
        if reply not in ('[ok]', 'ok', '0', '1'):
            raise RuntimeError(f'Jog stop failed: {reply!r}')
        return None

    @staticmethod
    def _gripper_id(gripper_id):
        if type(gripper_id) is not int or gripper_id != GRIPPER_ID:
            raise ValueError('This model controls only force gripper ID 14')
        return gripper_id

    def get_pro_gripper_angle(self, gripper_id=GRIPPER_ID):
        with self._api_lock:
            value = super().force_get_angle(self._gripper_id(gripper_id))
            if 0 <= value <= 100:
                self._stamps['gripper'] = self._received
            return value

    def set_pro_gripper_angle(self, angle, gripper_id=GRIPPER_ID):
        if type(angle) not in (int, float) or not math.isfinite(angle) or not 0 <= angle <= 100:
            raise ValueError('Gripper opening must be in 0..100')
        return super().force_set_angle(self._gripper_id(gripper_id), int(round(angle)))

    def get_pro_gripper_speed(self, gripper_id=GRIPPER_ID):
        return super().force_get_speed(self._gripper_id(gripper_id))

    def set_pro_gripper_speed(self, speed, gripper_id=GRIPPER_ID):
        if type(speed) is not int or not 1 <= speed <= 100:
            raise ValueError('Pro630 gripper SDK speed must be in 1..100')
        return super().force_set_speed(self._gripper_id(gripper_id), speed)

    def set_pro_gripper_enabled(self, enabled):
        if type(enabled) is not int or enabled not in (0, 1):
            raise ValueError('Gripper enable must be 0 or 1')
        return super().force_set_enabled(GRIPPER_ID, enabled)

    def set_motor_enabled(self, motors, enabled):
        """Use the documented Pro630 free-move mode for manual teaching.

        state_off may engage brakes, so release requests enter free-move
        without powering off. No undocumented servo-release or brake IO.
        """
        if motors != 254 or type(enabled) is not int or enabled not in (0, 1):
            raise ValueError('Drag mode may switch only all six arm joints')
        with self._api_lock:
            if self.is_power_on() != 1:
                raise RuntimeError('Pro630 must already be powered for drag following')
            # Drag locking means leaving free-move. Its acknowledged mode
            # request is sufficient; do not send state_on or gate on state_check.
            super().set_free_move_mode(not bool(enabled))
            if self._last_reply.strip().lower() not in ('1', '0', 'ok', '[ok]', 'true'):
                raise RuntimeError(f'Pro630 free-move command unconfirmed: {self._last_reply!r}')
            return 1

    def close(self):
        self._closed.set()
        sock = getattr(self, 'tcp_client', None)
        try:
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                sock.close()
        finally:
            self.is_client_started = False
            if self._lease is not None:
                lease, self._lease = self._lease, None
                lease.close()
