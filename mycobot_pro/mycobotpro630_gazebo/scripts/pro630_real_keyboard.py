"""Shared measured-pose reader and slider speed conversion."""
import math
import threading
import time


from pro630_model import ARM_FULL_SPEED_RAD_S, ARM_JOINT_SDK_SPEED_MAX
SDK_RAD_PER_SPEED = ARM_FULL_SPEED_RAD_S / ARM_JOINT_SDK_SPEED_MAX


class FeedbackUnavailable(RuntimeError):
    """A missing/invalid read pauses commands; it is not a hardware fault."""


class HardwareFault(RuntimeError):
    """A confirmed hardware fault must stay locked until operator recovery."""


class ArmReadError(FeedbackUnavailable):
    """A failed six-axis angle query or invalid angle payload."""


def valid_gripper_position(raw):
    """Check SDK read status before mapping 0..100 to the URDF coordinate."""
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise RuntimeError(f"Pro630 gripper read failed: invalid SDK response {raw!r}")
    if raw < 0:
        raise RuntimeError(
            f"Pro630 gripper read failed: SDK returned {raw!r}; "
            "Gazebo startup and real motion are blocked")
    if not math.isfinite(raw) or raw > 100:
        raise RuntimeError(f"Pro630 gripper read failed: invalid SDK position {raw!r}")
    return float(raw) / 100.0


def sdk_speed_for_rad(velocity, urdf_limit=3.14):
    if not math.isfinite(velocity) or not math.isfinite(urdf_limit) or urdf_limit <= 0:
        raise ValueError("invalid speed")
    # Round down, never exceed either the requested or the URDF speed.
    return max(0, min(ARM_JOINT_SDK_SPEED_MAX, int((min(abs(velocity), urdf_limit) + 1e-12) /
                             SDK_RAD_PER_SPEED)))



class RosPoseCache:
    """ROS-side joint cache. Only successful SDK reads are stored."""

    def __init__(self, with_gripper=True):
        self.with_gripper = with_gripper
        self.lock = threading.RLock()
        self.arm = None
        self.arm_time = None
        self.gripper = None
        self.gripper_time = None

    def ready(self):
        return self.arm is not None and (not self.with_gripper or self.gripper is not None)

    def store_arm(self, arm, when):
        with self.lock:
            if self.arm_time is None or when > self.arm_time:
                self.arm = list(arm)
                self.arm_time = when

    def store_gripper(self, value, when):
        with self.lock:
            self.gripper = float(value)
            self.gripper_time = when

    def pose(self):
        with self.lock:
            return list(self.arm) + ([self.gripper] if self.with_gripper else [])


class RealPoseReader:
    """Single-owner reads with a ROS-side pose cache; never writes to SDK.

    Arm angles come from the public sampler when a ROS proxy is used. While the robot is moving,
    the gripper opening comes from the cache and the slow gripper query is
    skipped. A gripper read of -1 is discarded and
    does not replace the cached opening. Other gripper failures still retry
    twice, then cool down for 1 s. Cache timestamps change only when a
    successful reading is stored.
    """
    supports_motion_cache = True

    def __init__(self, sdk, limits, clock=time.monotonic, report=None, with_gripper=True,
                 arm_max_age=0.5):
        self.with_gripper = with_gripper
        self.arm_max_age = arm_max_age
        self.sdk, self.limits, self.clock = sdk, limits, clock
        self.report = report or (lambda _: None)
        self.cache = RosPoseCache(with_gripper)
        self.gripper_valid = False
        self.gripper_rejected_minus_one = False
        self.gripper_from_cache = False
        self.next_gripper = 0.0
        self.gripper_period = 0.5
        self.gripper_max_age = 0.8
        self.retry_count = 0
        self.read_count = self.failure_count = self.success_count = 0
        self.last_raw = None
        self.last_error = ''
        self.arm_wall_time = None

    def _sample_stamp(self, kind):
        if hasattr(self.sdk, 'sample_time'):
            stamp = self.sdk.sample_time(kind)
            if stamp is None:
                raise FeedbackUnavailable(f"Pro630 {kind} sample has no receive timestamp")
            return stamp
        return self.clock(), time.time()

    def observe_arm(self, raw, when, wall_time):
        """Commit a received arm sample without waiting for a gripper query."""
        if not isinstance(raw, (list, tuple)) or len(raw) != 6:
            raise ArmReadError(f"Invalid Pro630 arm response: {raw!r}")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in raw):
            raise ArmReadError(f"Invalid Pro630 arm response: {raw!r}")
        arm = [math.radians(v) for v in raw]
        if not all(math.isfinite(v) and lo <= v <= hi
                   for v, (lo, hi) in zip(arm, self.limits[:6])):
                raise HardwareFault(f"Pro630 arm feedback violates joint limits: {raw!r}")
        with self.cache.lock:
            if self.cache.arm_time is None or when > self.cache.arm_time:
                self.cache.store_arm(arm, when)
                self.arm_wall_time = wall_time
            return self.cache.pose() if self.cache.ready() else None

    @property
    def arm(self):
        return None if self.cache.arm is None else list(self.cache.arm)

    @property
    def arm_time(self):
        return self.cache.arm_time

    @property
    def gripper(self):
        return self.cache.gripper

    @property
    def gripper_time(self):
        return self.cache.gripper_time

    def fresh(self):
        now = self.clock()
        if self.cache.arm_time is None or now - self.cache.arm_time > self.arm_max_age:
            return False
        if not self.with_gripper:
            return True
        if self.gripper_from_cache:
            try:
                self.cached_gripper()
                return True
            except RuntimeError:
                return False
        if not self.gripper_valid or self.cache.gripper is None:
            return False
        # An invalid idle reply must not extend the last measured opening's age.
        return now - self.cache.gripper_time <= self.gripper_max_age

    def force_gripper_read(self):
        if not self.with_gripper:
            raise RuntimeError("Gripper is not installed")
        self.next_gripper = self.clock()

    def cached_gripper(self):
        """Last successful measured opening; no SDK call or age refresh."""
        if not self.with_gripper:
            raise RuntimeError("Gripper is not installed")
        with self.cache.lock:
            value = self.cache.gripper
            if (value is None or self.cache.gripper_time is None or
                    not math.isfinite(value) or not 0.0 <= value <= 1.0):
                raise FeedbackUnavailable("No valid gripper angle is available in cache")
            return float(value)

    def read_fresh_gripper(self):
        """One successful hardware read; never satisfy a preflight with cache."""
        if not self.with_gripper:
            raise RuntimeError("Gripper is not installed")
        raw = self.sdk.get_pro_gripper_angle(gripper_id=14)
        value = valid_gripper_position(raw)
        stamp, _ = self._sample_stamp('gripper')
        if self.clock() - stamp > self.gripper_max_age:
            raise RuntimeError("new gripper response is stale")
        self.cache.store_gripper(value, stamp)
        self.gripper_valid = True
        self.gripper_rejected_minus_one = False
        self.gripper_from_cache = False
        self.next_gripper = self.clock() + self.gripper_period
        return value

    def __call__(self, moving=False):
        try:
            raw = self.sdk.get_angles()
        except Exception as exc:
            raise ArmReadError(f'Pro630 arm read failed: {exc}') from exc
        # Validate the payload before querying its timestamp: an invalid reply
        # must count as an angle-read failure rather than a timestamp fault.
        if (not isinstance(raw, (list, tuple)) or len(raw) != 6 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in raw)):
            raise ArmReadError(f'Invalid Pro630 arm response: {raw!r}')
        if not all(math.isfinite(v) and lo <= math.radians(v) <= hi
                   for v, (lo, hi) in zip(raw, self.limits[:6])):
            raise HardwareFault(f'Pro630 arm feedback violates joint limits: {raw!r}')
        stamp, wall = self._sample_stamp('arm')
        self.observe_arm(raw, stamp, wall)
        if not self.with_gripper:
            return self._combined()
        if moving and self.cache.gripper is not None:
            self.cached_gripper()
            self.gripper_from_cache = True
            # Use a known valid historical sample even after an idle read
            # fails. Do not turn that failure into a new pre-motion query.
            return self._combined()
        self.gripper_from_cache = False
        if self.clock() >= self.next_gripper:
            started = self.clock()
            self.read_count += 1
            self.last_raw = None
            try:
                self.last_raw = self.sdk.get_pro_gripper_angle(gripper_id=14)
                if self.last_raw == -1:
                    if self.cache.gripper is None:
                        raise RuntimeError(
                            "Pro630 gripper read failed: SDK returned -1; "
                            "Gazebo startup and real motion are blocked")
                    # Do not store -1. Keep the last successful opening.
                    self.gripper_rejected_minus_one = True
                    self.retry_count = 0
                    self.next_gripper = self.clock() + self.gripper_period
                    self.report(
                        f"gripper read returned -1; cache unchanged at "
                        f"{self.cache.gripper:.3f}, "
                        f"read_elapsed={self.clock() - started:.3f}s")
                    return self._combined()
                value = valid_gripper_position(self.last_raw)
            except Exception as exc:
                usable_cache = (self.cache.gripper is not None and
                                self.cache.gripper_time is not None and
                                self.clock() - self.cache.gripper_time <= self.gripper_max_age)
                self.gripper_valid = usable_cache
                self.gripper_rejected_minus_one = False
                self.failure_count += 1
                self.retry_count += 1
                attempt = self.retry_count
                delay = 0.2 if attempt <= 2 else 1.0
                self.next_gripper = self.clock() + delay
                if attempt >= 3:
                    self.retry_count = 0
                self.last_error = (
                    f"{exc}; raw={self.last_raw!r}, read_elapsed="
                    f"{self.clock() - started:.3f}s, failed_attempt={attempt}/3, "
                    f"total_failures={self.failure_count}, next_read_in={delay:.1f}s")
                self.report(self.last_error)
                if usable_cache:
                    return self._combined()
                raise FeedbackUnavailable('Gripper feedback unavailable: ' + self.last_error) from exc
            self.cache.store_gripper(value, self._sample_stamp('gripper')[0])
            self.gripper_valid = True
            self.gripper_rejected_minus_one = False
            self.retry_count = 0
            self.success_count += 1
            self.next_gripper = self.clock() + self.gripper_period
            self.last_error = ''
            self.report(
                f"gripper read valid: raw={self.last_raw!r}, "
                f"read_elapsed={self.clock() - started:.3f}s, reads={self.read_count}, "
                f"total_failures={self.failure_count}")
        return self._combined()

    def seed_gripper(self, value):
        if not self.with_gripper:
            raise RuntimeError("Gripper is not installed")
        self.cache.store_gripper(float(value), self._sample_stamp('gripper')[0])
        self.gripper_valid = True
        self.gripper_rejected_minus_one = False
        self.next_gripper = self.clock() + self.gripper_period

    def _combined(self):
        if not self.fresh():
            if self.cache.arm_time is None or self.clock() - self.cache.arm_time > self.arm_max_age:
                reason = f'Six-axis feedback is missing or older than {self.arm_max_age:g} s'
            else:
                reason = 'Gripper feedback is unavailable or older than 0.8 s'
            raise FeedbackUnavailable(reason)
        return self.cache.pose()
