"""Cartesian jog/pose worker with independent priority arm STOP requests."""
from collections import deque
from contextlib import nullcontext
import math
import threading
import time
from pro630_configuration import LIMITS_DEG
from pro630_model import HOME_MODEL_ANGLES_DEG, HARDWARE_OFFSETS_DEG, ARM_JOINT_SDK_SPEED_MAX, ARM_CARTESIAN_SDK_SPEED_MAX, KEYBOARD_ARM_FEEDBACK_MAX_AGE_S
from pro630_broker_support import KeyboardCommandCancelled, PublicArmFeedbackUnavailable


CARTESIAN_AXES = ('X', 'Y', 'Z', 'RX', 'RY', 'RZ', 'Gripper')
SPEED_GEARS = (6, 12, 18, 24, 30)  # Native SDK percentages, not physical units.
GRIPPER_SPEED_GEARS = (4, 8, 12, 16, 20)
LOW_Z_MM = 20.0  # Base-frame TCP height, including the installed tool offset.
SINGULAR_WINDOW_S = 0.2
SINGULAR_FIRST_WINDOW_S = 0.8
SINGULAR_JOINT_DELTA_DEG = 0.2
SINGULAR_MOTION_DELTA_DEG = 0.1
SINGULAR_POSE_MESSAGE = 'The current pose may be singular. Please use joint control to move away.'
COORD_POLL_PERIOD = 1.0 / 20.0
WORKER_MAX_WAIT_S = 0.012  # Maximum scheduler wait; coordinate polling stays at 20 Hz.
INIT_POSE_DEG = (0.,) * 6  # Model zero maps to hardware [0, -90, 0, -90, 0, 0].
DEFAULT_HOME_POSE_DEG = HOME_MODEL_ANGLES_DEG


class CartesianKeyboardTransport:
    def __init__(self, sdk, with_gripper=True, clock=time.monotonic, report=None, allow_collision=False,
                 pose_validator=None):
        self.sdk, self.with_gripper, self.clock = sdk, with_gripper, clock
        self.pose_validator = pose_validator
        from pro630_configuration import collision_policy
        self.allow_collision = collision_policy(allow_collision)
        self.report = report or (lambda text: None)
        self.lock = threading.RLock()
        self.shutdown = threading.Event()
        self.armed = False
        self.gear = 2
        self.desired = self.active = None
        self.home_pose = DEFAULT_HOME_POSE_DEG
        self.pose_request = self.pose_active = None
        self.save_home_requested = False
        self.epoch = 0
        self.last_keepalive = self.clock()
        self.last_feedback = float('-inf')
        self.coords = None
        self.low_z = False
        self.next_feedback = self.next_error = 0.
        self.arm_requested = False
        self.stop_pending = self.force_stop = False
        self.waiting_generation = 0
        self.notices = deque()
        self.priority_enabled = callable(getattr(sdk, 'priority_stop', None))
        self.stop_requested = self.stop_completed = 0
        self.priority_error = None
        self.priority_exit = threading.Event()
        self._singular_samples = deque()
        self._singular_jog = None
        self._singular_moving = False
        self._singular_locked = False

    def notice(self, text, level='info'):
        with self.lock:
            self.notices.append((text, level))

    def drain_notices(self):
        with self.lock:
            result = list(self.notices)
            self.notices.clear()
            return result

    def _queue_priority_stop(self, force=False):
        # Called under self.lock. The normal worker may be blocked in a query.
        if self.priority_enabled and (force or self.pose_active is not None or
                                     (self.active is not None and self.active[0] < 6)):
            # No new motion is dispatched while STOP is pending. One queued
            # STOP therefore covers repeated releases/cancellation of that move.
            if self.stop_requested == self.stop_completed:
                self.stop_requested += 1

    def _priority_loop(self):
        while not self.priority_exit.is_set():
            with self.lock:
                if (self.armed and (self.active is not None or self.pose_active is not None)
                        and self.clock() - self.last_keepalive > .5):
                    self.stop()
                    self.waiting_generation += 1
                    self.notice('Keyboard input heartbeat lost; controls locked.', 'warning')
                sequence = self.stop_requested
                pending = sequence > self.stop_completed
            if pending:
                with self.lock:
                    stopping = self.active
                jog_axis = stopping[0] if stopping is not None and stopping[0] < 6 else None
                try:
                    self.accepted(self.sdk.priority_stop(jog_axis), stop=True)
                    with self.lock:
                        # Publish completion with the stopped-state update so a
                        # subsequent independent STOP cannot join a finished one.
                        self.stop_completed = sequence
                        # A gripper requires a measured-opening hold in the normal worker.
                        if self.active is None or self.active[0] < 6:
                            self.active = None
                        self.pose_active = None
                        if sequence == self.stop_requested:
                            self.force_stop = False
                            self.stop_pending = self.active is not None
                        singular = self._singular_locked
                    self.notice(SINGULAR_POSE_MESSAGE if singular else 'STOP sent.',
                                'warning' if singular else 'info')
                except Exception as exc:
                    with self.lock:
                        self.priority_error = exc
                finally:
                    with self.lock:
                        self.stop_completed = sequence
            self.priority_exit.wait(.01)

    def request_arm(self):
        with self.lock:
            self._singular_locked = False
            self._queue_priority_stop()
            self.armed = False
            self.arm_requested = True
            self.stop_pending = True
            self.desired = None
            self.pose_request = None
            self.save_home_requested = False
            self.epoch += 1
        return 'Control unlock requested.'

    def request(self, axis, direction):
        with self.lock:
            if type(axis) is not int or not 0 <= axis < (7 if self.with_gripper else 6) or direction not in (-1, 1):
                return 'rejected: invalid coordinate control'
            if not self.armed:
                return 'rejected: controls locked; press M to unlock'
            if self.low_z and (axis, direction) != (2, 1):
                return 'rejected: only X is allowed'
            if (self.pose_request is not None or self.pose_active is not None or
                    self.save_home_requested):
                return 'rejected: a pose command is in progress'
            self.desired = (axis, direction, self.speed_for_axis(axis))
            self.last_keepalive = self.clock()
            self.epoch += 1
        return 'requested: native SDK jog'

    def request_pose(self, name):
        with self.lock:
            if name not in ('Init', 'Home'):
                return 'rejected: invalid pose command'
            if not self.armed:
                return 'rejected: controls locked; press M to unlock'
            if self.low_z:
                return 'rejected: only X is allowed'
            if not self.idle():
                return 'rejected: stop the current command before requesting a pose'
            target = INIT_POSE_DEG if name == 'Init' else self.home_pose
            self.pose_request = (name, tuple(target), SPEED_GEARS[self.gear - 1])
            self.last_keepalive = self.clock()
            self.epoch += 1
        return f'requested: {name} pose'

    def request_save_home(self):
        with self.lock:
            if not self.idle() or self.arm_requested:
                return 'rejected: stop the current command before saving Home'
            self.save_home_requested = True
            self.epoch += 1
        return 'Home save requested.'

    def home_snapshot(self):
        with self.lock:
            return tuple(self.home_pose)

    def release(self):
        with self.lock:
            self._queue_priority_stop()
            self.desired = None
            self.pose_request = None
            self.save_home_requested = False
            self.stop_pending = True
            self.epoch += 1

    def stop(self):
        with self.lock:
            self._queue_priority_stop(force=True)
            self.armed = False
            self.arm_requested = False
            self.desired = None
            self.pose_request = None
            self.save_home_requested = False
            self.stop_pending = self.force_stop = True
            self.epoch += 1

    def keepalive(self):
        with self.lock:
            self.last_keepalive = self.clock()

    def set_gear(self, gear):
        with self.lock:
            self.gear = max(1, min(5, int(gear)))
            if self.desired is not None:
                self.desired = (*self.desired[:2], self.speed_for_axis(self.desired[0]))
                self.epoch += 1

    def speed_for_axis(self, axis):
        gears = GRIPPER_SPEED_GEARS if axis == 6 else SPEED_GEARS
        return gears[self.gear - 1]

    def idle(self):
        with self.lock:
            return (self.active is None and self.desired is None and self.pose_request is None
                    and self.pose_active is None and not self.save_home_requested)

    @staticmethod
    def accepted(result, gripper=False, stop=False):
        if gripper:
            valid = type(result) is int and result == 1
        else:
            valid = result is None or type(result) is int and result in (0, 1)
        if not valid:
            raise RuntimeError(f'SDK rejected control: {result!r}')

    def _stop_active(self, force=False):
        with self.lock:
            active = self.active
            pose_active = self.pose_active
            if (self.priority_enabled and not self.priority_exit.is_set()
                    and (active is None or active[0] < 6)
                    and (self.stop_requested > self.stop_completed or force
                         or active is not None or pose_active is not None)):
                # One priority worker owns arm STOP, including key changes
                # noticed after a delayed reply. Never send a parallel STOP RPC.
                self._queue_priority_stop(force=True)
                return False
        if active is not None and active[0] == 6:
            opening = self.sdk.get_pro_gripper_angle()
            if type(opening) not in (int, float) or not math.isfinite(opening) or not 0 <= opening <= 100:
                raise RuntimeError('Cannot hold gripper: valid measured opening unavailable')
            self.accepted(self.sdk.set_pro_gripper_angle(int(round(opening))), gripper=True)
            self.notice('Gripper hold sent.')
        elif active is not None or pose_active is not None or force:
            self.accepted(self.sdk.stop(deceleration=0, _async=True), stop=True)
            self.notice('STOP sent.')
        with self.lock:
            self.active = None
            self.pose_active = None
        return True

    def _fault(self, exc):
        with self.lock:
            self.armed = False
            self.arm_requested = False
            self.desired = None
            self.pose_request = None
            self.save_home_requested = False
            self.epoch += 1
            self.waiting_generation += 1
        try:
            self._stop_active()
        except Exception as stop_error:
            self.notice(f'STOP/hold failed: {stop_error}', 'error')
        reason = str(exc)
        self.notice(f'Control fault (locked): {reason}', 'error')

    def _read_coords(self):
        started = self.clock()
        values = self.sdk.get_coords()
        if (not isinstance(values, (list, tuple)) or len(values) != 6 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise RuntimeError('SDK coordinate feedback unavailable')
        with self.lock:
            self.coords = tuple(values)
            self.last_feedback = self.clock()
            # Schedule from request start, not reply completion: otherwise SDK
            # latency is added to every 50 ms period. Calls never overlap.
            self.next_feedback = started + COORD_POLL_PERIOD
            was_low = self.low_z
            self.low_z = not self.allow_collision and values[2] <= LOW_Z_MM
            entered = self.low_z and not was_low
            if entered:
                # Latch the entry, not every low sample: M may subsequently
                # unlock an upward escape while feedback is still below 20 mm.
                self.armed = False
                self.arm_requested = False
                self.desired = None
                self.pose_request = None
                self.save_home_requested = False
                self.epoch += 1
                self.waiting_generation += 1
                self.notice('Locked. Press M.', 'warning')
            elif was_low and not self.low_z:
                self.notice('All motion keys available.' if self.armed else 'Locked. Press M.')
        if entered:
            self._stop_active()

    def _read_error(self):
        error = self.sdk.get_error_information()
        if error != 0:
            raise RuntimeError(str(error) if isinstance(error, str) else 'SDK controller status unavailable')

    @staticmethod
    def _angles(values):
        if (not isinstance(values, (list, tuple)) or len(values) != 6 or
                any(type(v) not in (int, float) or not math.isfinite(v) or not lo <= v <= hi
                    for v, (lo, hi) in zip(values, LIMITS_DEG))):
            raise RuntimeError('Fresh six-axis joint feedback unavailable')
        return tuple(float(v) for v in values)

    def _save_home(self):
        with self.lock:
            if not self.save_home_requested:
                return
            epoch = self.epoch
        try:
            target = self._angles(self.sdk.get_angles())
            with self.lock:
                if epoch == self.epoch and self.save_home_requested and not self.shutdown.is_set():
                    self.home_pose = target
                    self.report(f'Home pose saved (deg): {[v-o for v,o in zip(target, HARDWARE_OFFSETS_DEG)]}; session only.')
                    self.notice('Home saved for this session.')
        except Exception as exc:
            self._fault(f'Home pose not saved: {exc}')
        finally:
            with self.lock:
                if epoch == self.epoch:
                    self.save_home_requested = False

    def _motion_context(self):
        factory = getattr(self.sdk, 'keyboard_motion', None)
        return factory() if callable(factory) else nullcontext()

    def _start_pose(self):
        with self._motion_context():
            self._start_pose_command()

    def _start_pose_command(self):
        with self.lock:
            command = self.pose_request
            if command is None or not self.armed or self.low_z or self.shutdown.is_set():
                return
            epoch = self.epoch
        name, target, speed = command
        try:
            measured = self._angles(self.sdk.get_angles())
        except PublicArmFeedbackUnavailable:
            # The preflight must still obtain a fresh starting pose before
            # dispatch. Arrival monitoring retains its public-owner fallback.
            measured = None
        def cancelled():
            with self.lock:
                return (epoch != self.epoch or self.shutdown.is_set() or not self.armed
                        or self.clock() - self.last_keepalive > .5)
        self.notice(f'{name}: validating joint path for collisions...')
        valid, reason = ((False, 'Joint collision validation is unavailable.')
                         if self.pose_validator is None else self.pose_validator(target, cancelled))
        with self.lock:
            if cancelled():
                if epoch == self.epoch and not self.shutdown.is_set():
                    self.stop()
                    self.notice('Keyboard input heartbeat lost; controls locked.', 'warning')
                return
            if not valid:
                self.pose_request = None
                self.notice(f'{name} rejected: {reason}', 'warning')
                return
            self.pose_request = None
            now = self.clock()
            self.pose_active = dict(name=name, target=target,
                                    deadline=now+600., next_check=now+.25,
                                    measured=measured, progress_at=now)
        self.accepted(self.sdk.send_angles(list(target), max(1, round(speed * ARM_JOINT_SDK_SPEED_MAX / 100)), _async=True))
        self.report(f'{name} pose requested (deg): {[v-o for v,o in zip(target, HARDWARE_OFFSETS_DEG)]}; SDK speed {speed}%.')
        self.notice(f'{name} pose sent.')
        with self.lock:
            obsolete = epoch != self.epoch or self.shutdown.is_set()
        if obsolete:
            self._stop_active()

    def _monitor_pose(self):
        with self.lock:
            pose, epoch = self.pose_active, self.epoch
            if pose is None:
                return
            now = self.clock()
            if now > pose['deadline']:
                raise RuntimeError(f"{pose['name']} pose timed out waiting for arrival")
            if now < pose['next_check']:
                return
            pose['next_check'] = now+.2
        try:
            measured = self._angles(self.sdk.get_angles())
        except PublicArmFeedbackUnavailable:
            # Query through the existing public SDK owner, never open another
            # connection or report completion from stale subscription values.
            moving = self.sdk.is_moving()
            if type(moving) is not int or moving not in (0, 1):
                raise RuntimeError('SDK pose motion status unavailable')
            if moving:
                pose.pop('cache_idle_since', None)
                return
            self._read_error()
            arrived = self.sdk.is_in_position(list(pose['target']), 0)
            if type(arrived) is not int or arrived not in (0, 1):
                raise RuntimeError('SDK pose target confirmation unavailable')
            with self.lock:
                if epoch != self.epoch or self.pose_active is not pose or self.shutdown.is_set():
                    return
                if arrived:
                    self.pose_active = None
                    self.notice(f"{pose['name']} pose complete: measured joint angles confirmed.")
                elif now - pose.setdefault('cache_idle_since', now) >= 3.:
                    raise RuntimeError(f"{pose['name']} pose not completed: target not reached")
            return
        pose.pop('cache_idle_since', None)
        if pose['measured'] is None or max(abs(a-b) for a,b in zip(measured, pose['measured'])) >= .05:
            pose['measured'], pose['progress_at'] = measured, now
        arrived = max(abs(a-b) for a,b in zip(measured, pose['target'])) <= .5
        if 'idle_confirmed_at' in pose:
            stamp = self.sdk.sample_time('arm')
            if (not isinstance(stamp, (list, tuple)) or not stamp or
                    type(stamp[0]) not in (int, float) or not math.isfinite(stamp[0]) or
                    self.clock() - stamp[0] > KEYBOARD_ARM_FEEDBACK_MAX_AGE_S):
                raise RuntimeError('Fresh six-axis joint feedback unavailable after idle confirmation')
            if stamp[0] <= pose['idle_confirmed_at']:
                return
            with self.lock:
                if epoch != self.epoch or self.pose_active is not pose or self.shutdown.is_set():
                    return
                if not arrived:
                    raise RuntimeError(f"{pose['name']} pose not completed: target not reached")
                self.pose_active = None
                self.notice(f"{pose['name']} pose complete: measured joint angles confirmed.")
            return
        if arrived or now - pose['progress_at'] >= 3.:
            moving = self.sdk.is_moving()
            if type(moving) is not int or moving not in (0, 1):
                raise RuntimeError('SDK pose motion status unavailable')
            if moving == 0:
                self._read_error()
            with self.lock:
                if epoch != self.epoch or self.pose_active is not pose or self.shutdown.is_set():
                    return
                if moving == 0:
                    # Angles read before the blocking status queries cannot confirm arrival.
                    pose['idle_confirmed_at'] = self.clock()
                    pose['next_check'] = self.clock() + .05
                elif not arrived:
                    pose['progress_at'] = now

    def _dispatch_desired(self, desired, epoch):
        with self._motion_context():
            with self.lock:
                if epoch != self.epoch or self.shutdown.is_set():
                    return
                send = self.active != desired
                if send:
                    if self.low_z and desired[:2] != (2, 1):
                        self.desired = None
                        return
                    self.active = desired  # Ambiguous writes must still be stopped.
            if send:
                axis, direction, speed = desired
                if axis < 6:
                    result = self.sdk.jog_coord(axis + 1, int(direction > 0), max(1, round(speed * ARM_CARTESIAN_SDK_SPEED_MAX / 100)), _async=True)
                    self.accepted(result)
                else:
                    self.accepted(self.sdk.set_pro_gripper_speed(speed))
                    with self.lock:
                        if epoch != self.epoch or self.shutdown.is_set():
                            self.active = None  # No opening command was sent.
                            return
                    self.accepted(self.sdk.set_pro_gripper_angle(100 if direction > 0 else 0), gripper=True)
                self.notice(f'{CARTESIAN_AXES[axis]}: SDK jog command sent at {speed}%.')
            with self.lock:
                obsolete = epoch != self.epoch or self.shutdown.is_set()
            if obsolete:
                self._stop_active()
                return

    def _remember_singular_sample(self, when, angles):
        samples = self._singular_samples
        if samples and samples[-1][0] == when:
            return None
        samples.append((when, tuple(angles)))
        window = SINGULAR_WINDOW_S if self._singular_moving else SINGULAR_FIRST_WINDOW_S
        if self._singular_moving:
            while len(samples) >= 3 and when - samples[1][0] >= window:
                samples.popleft()
        for previous in reversed(samples):
            if when - previous[0] >= window:
                return previous
        return None

    def _watch_singular_jog(self):
        """Public joint cache only. A held Cartesian jog that does not move is singular."""
        with self.lock:
            active, desired, armed = self.active, self.desired, self.armed
        if not armed or active is None or active[0] >= 6 or desired != active:
            self._singular_samples.clear()
            self._singular_jog = None
            self._singular_moving = False
            return
        jog = active[:2]
        if jog != self._singular_jog:
            self._singular_jog = jog
            self._singular_samples.clear()
            self._singular_moving = False
        try:
            angles = self.sdk.get_angles()
            stamp = self.sdk.sample_time('arm')
        except PublicArmFeedbackUnavailable:
            return
        if (not isinstance(stamp, (list, tuple)) or not stamp or
                type(stamp[0]) not in (int, float) or not math.isfinite(stamp[0]) or
                len(angles) != 6 or any(type(v) not in (int, float) or not math.isfinite(v) for v in angles)):
            return
        previous = self._remember_singular_sample(stamp[0], angles)
        if previous is None:
            return
        limit = SINGULAR_MOTION_DELTA_DEG if self._singular_moving else SINGULAR_JOINT_DELTA_DEG
        if max(abs(a - b) for a, b in zip(angles, previous[1])) >= limit - 1e-6:
            self._singular_moving = True
            return
        self._singular_samples.clear()
        self._singular_jog = None
        self._singular_moving = False
        with self.lock:
            self._singular_locked = True
        self.stop()

    def cycle(self):
        with self.lock:
            if self.stop_requested > self.stop_completed:
                return
            if self.priority_error is not None:
                error, self.priority_error = self.priority_error, None
                raise RuntimeError(f'Priority STOP failed: {error}')
            stop, force = self.stop_pending, self.force_stop
            self.stop_pending = self.force_stop = False
            arm, epoch = self.arm_requested, self.epoch
            self.arm_requested = False
        if stop and not self._stop_active(force):
            return
        if arm:
            self._read_coords()
            self._read_error()
            with self.lock:
                if epoch == self.epoch and not self.shutdown.is_set():
                    self.armed = True
                    self.notice('Unlocked: X only.' if self.low_z else 'Unlocked.')
        else:
            # Read height before dispatch, so a newly observed low-zone entry
            # cancels an already queued command before any new SDK jog write.
            now = self.clock()
            if now >= self.next_feedback:
                self.next_feedback = now + COORD_POLL_PERIOD
                try:
                    self._read_coords()
                except Exception:
                    if self.armed and self.clock() - self.last_feedback > 1.:
                        raise
        with self.lock:
            # A release can arrive during a blocking feedback query. Wait for
            # its existing STOP before considering a newly pressed direction.
            if self.stop_requested > self.stop_completed:
                return
            if (self.desired is not None or self.pose_request is not None or self.pose_active is not None) and self.clock() - self.last_keepalive > .5:
                self.desired = None
                self.pose_request = None
                self.armed = False
                self.epoch += 1
                self.waiting_generation += 1
                self.notice('Keyboard input heartbeat lost; controls locked.', 'warning')
            desired = self.desired if self.armed else None
            epoch = self.epoch
            active = self.active
            stop_pose = self.pose_active is not None and not self.armed
        if active is not None and active != desired:
            if not self._stop_active():
                return
        if stop_pose and not self._stop_active():
            return
        if desired is not None:
            self._dispatch_desired(desired, epoch)
        self._watch_singular_jog()
        self._save_home()
        self._start_pose()
        self._monitor_pose()
        now = self.clock()
        if (self.active is not None or self.pose_active is not None) and now >= self.next_error:
            self.next_error = now + .5
            self._read_error()

    def run(self):
        priority = None
        if self.priority_enabled:
            priority = threading.Thread(target=self._priority_loop, daemon=True)
            priority.start()
        try:
            while not self.shutdown.is_set():
                try:
                    self.cycle()
                except KeyboardCommandCancelled:
                    # The release STOP already invalidated this write. stop() would
                    # lock controls and force another STOP, so the next key needs M.
                    pass
                except Exception as exc:
                    self._fault(exc)
                # Shorten scheduler latency without increasing coordinate RPC
                # frequency or changing the UI key-release/debounce timers.
                self.shutdown.wait(min(WORKER_MAX_WAIT_S, max(.001, self.next_feedback - self.clock())))
        finally:
            self.priority_exit.set()
            if priority is not None:
                priority.join(timeout=8.)
            try:
                self._stop_active(force=True)
            except Exception as exc:
                self.notice(f'Exit STOP/hold failed: {exc}', 'error')
