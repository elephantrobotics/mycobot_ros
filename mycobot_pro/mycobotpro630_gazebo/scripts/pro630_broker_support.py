"""Exclusive feature leases and a closed SDK method allowlist."""
import threading
import time
from contextlib import contextmanager

KEYBOARD_MOTION_METHODS = {'jog_coord', 'send_angles', 'set_pro_gripper_angle', 'set_pro_gripper_speed'}
KEYBOARD_CANCELLED = 'Keyboard command cancelled by STOP.'


class PublicArmFeedbackUnavailable(RuntimeError):
    """Keyboard subscription cache has no fresh six-axis sample."""


class KeyboardCommandCancelled(RuntimeError):
    def __init__(self):
        super().__init__(KEYBOARD_CANCELLED)


class KeyboardStopBarrier:
    """Invalidate queued keyboard writes; hold the gate only during actual writes."""
    def __init__(self):
        self.lock = threading.RLock()
        self.generation = 0
        self.stopping = False
        self.local = threading.local()

    def check(self, generation):
        if type(generation) is not int:
            raise ValueError('Keyboard motion generation is required')
        if generation != self.generation:
            raise KeyboardCommandCancelled()
        if self.stopping:
            raise RuntimeError('STOP has not been confirmed.')

    @contextmanager
    def guard(self, generation):
        with self.lock:
            self.check(generation)
            yield

    @contextmanager
    def command(self, generation):
        with self.guard(generation):
            pass
        previous = getattr(self.local, 'generation', None)
        self.local.generation = generation
        try:
            yield
            with self.guard(generation):
                pass
        finally:
            self.local.generation = previous

    @contextmanager
    def write(self):
        with self.lock:
            generation = getattr(self.local, 'generation', None)
            if generation is not None:
                self.check(generation)
            yield

    def begin_stop(self):
        with self.lock:
            self.generation += 1
            self.stopping = True
            return self.generation

    def finish_stop(self, generation):
        with self.lock:
            if generation == self.generation:
                self.stopping = False

READS = {'get_angles', 'is_power_on', 'is_moving', 'get_error_information',
         'get_pro_gripper_angle', 'get_pro_gripper_speed'}
WRITES = {
    'gui': {'send_angles', 'set_pro_gripper_angle', 'set_pro_gripper_speed', 'stop',
            'get_coords', 'get_pro_gripper_status', 'send_coords', 'is_in_position',
            'get_motion_status'},
    'slider': {'send_angles', 'set_pro_gripper_angle', 'set_pro_gripper_speed', 'stop'},
    'keyboard': {'jog_coord', 'get_coords', 'send_angles', 'set_pro_gripper_angle',
                 'set_pro_gripper_speed', 'stop', 'keyboard_stop'},
    'drag': {'set_motor_enabled', 'set_pro_gripper_enabled'},
}


class FeatureLease:
    def __init__(self, clock=time.monotonic, timeout=3.0):
        self.clock, self.timeout = clock, timeout
        self.lock = threading.RLock()
        self.token = None
        self.feature = None
        self.seen = 0.0

    def claim(self, token, feature):
        with self.lock:
            if feature not in WRITES or not token:
                raise ValueError('Invalid control feature')
            if self.token is not None and token != self.token:
                raise RuntimeError('Another feature owns the public robot connection')
            self.token, self.feature, self.seen = token, feature, self.clock()

    def touch(self, token):
        with self.lock:
            if self.token != token or self.token is None:
                raise RuntimeError('Control ownership expired; restart the feature launch')
            self.seen = self.clock()

    def authorize(self, token, method):
        with self.lock:
            self.touch(token)
            if method not in READS | WRITES[self.feature]:
                raise ValueError(f'{method} is not allowed for {self.feature}')

    def expired(self):
        with self.lock:
            return self.token is not None and self.clock() - self.seen > self.timeout

    def clear(self, token):
        with self.lock:
            if token != self.token:
                raise RuntimeError('Wrong feature owner')
            self.token = self.feature = None
