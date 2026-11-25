"""Delayed interpolation of measured Pro630 poses for Gazebo rendering.

The render time always stays one estimated sample period behind the newest
measurement, so Gazebo never runs ahead of the real robot.
"""
import math
import threading
import time


POSE_SIZE = 7
MAX_SAMPLES = 64
INITIAL_PERIOD_S = 0.15
MIN_PERIOD_S = 0.05
MAX_PERIOD_S = 0.5
MIN_PERIOD_SAMPLE_S = 0.02
MAX_PERIOD_SAMPLE_S = 0.6
RENDER_MARGIN_S = 0.03
MAX_INTERP_GAP_S = 0.6
MAX_SAMPLE_AGE_S = 0.5


class RealPoseBuffer:
    """Thread-safe pose history. The SDK thread pushes; the renderer reads."""

    def __init__(self, clock=time.monotonic, pose_size=POSE_SIZE,
                 max_sample_age=MAX_SAMPLE_AGE_S):
        self.pose_size = pose_size
        self.max_sample_age = max_sample_age
        self._clock = clock
        self._lock = threading.Lock()
        self._samples = []
        self._period = INITIAL_PERIOD_S

    def push(self, pose, when=None):
        if len(pose) != self.pose_size or not all(math.isfinite(value) for value in pose):
            raise ValueError("real pose has an invalid size or non-finite joint values")
        stamp = self._clock() if when is None else float(when)
        if not math.isfinite(stamp):
            raise ValueError("real pose time must be finite")
        stored = (stamp, [float(value) for value in pose])
        with self._lock:
            if self._samples:
                interval = stamp - self._samples[-1][0]
                if MIN_PERIOD_SAMPLE_S <= interval <= MAX_PERIOD_SAMPLE_S:
                    self._period = min(MAX_PERIOD_S, max(
                        MIN_PERIOD_S, 0.7 * self._period + 0.3 * interval))
            self._samples.append(stored)
            if len(self._samples) > MAX_SAMPLES:
                del self._samples[:-MAX_SAMPLES]

    def render(self, now=None):
        """Return (pose, velocity) at one sample period behind now, or None."""
        with self._lock:
            if not self._samples:
                return None
            current = self._clock() if now is None else float(now)
            target_time = current - (self._period + RENDER_MARGIN_S)
            newest_time, newest = self._samples[-1]
            if current - newest_time > self.max_sample_age:
                return None
            if target_time >= newest_time:
                return list(newest), [0.0] * self.pose_size
            oldest_time, oldest = self._samples[0]
            if target_time <= oldest_time:
                return list(oldest), [0.0] * self.pose_size
            previous = self._samples[0]
            for sample in self._samples[1:]:
                if sample[0] >= target_time:
                    left_time, left = previous
                    right_time, right = sample
                    if right_time - left_time > MAX_INTERP_GAP_S:
                        return list(right), [0.0] * self.pose_size
                    span = right_time - left_time
                    fraction = (target_time - left_time) / span
                    pose = [
                        start + (end - start) * fraction
                        for start, end in zip(left, right)
                    ]
                    velocity = [(end - start) / span for start, end in zip(left, right)]
                    return pose, velocity
                previous = sample
            return list(newest), [0.0] * self.pose_size

    def latest(self):
        with self._lock:
            if not self._samples:
                return None
            stamp, pose = self._samples[-1]
            return stamp, list(pose)

    def clear(self):
        with self._lock:
            self._samples.clear()
            self._period = INITIAL_PERIOD_S
