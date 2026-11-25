"""Hardware-independent state and newest-frame storage for real drag following."""
import math
import threading
import time


def set_arm_enabled(sdk, enabled):
    """254 addresses the six arm motors; never use the separate gripper API."""
    result = sdk.set_motor_enabled(254, int(enabled))
    if isinstance(result, bool) or result != 1:
        raise RuntimeError(f'Arm motor command was not acknowledged (result={result!r}); state unknown')


def set_drag_enabled(sdk, enabled, with_gripper=True):
    """Switch six-axis and gripper enable together; partial failure is unknown."""
    set_arm_enabled(sdk, enabled)
    if not with_gripper:
        return
    result = sdk.set_pro_gripper_enabled(int(enabled))
    if isinstance(result, bool) or result != 1:
        raise RuntimeError(f'Arm command acknowledged, but gripper command was not acknowledged '
                           f'(result={result!r}); combined state unknown')


class DragState:
    def __init__(self, sdk, enabled=True, clock=time.monotonic, with_gripper=True, report=None):
        self.report = report or (lambda text, level='info': None)
        self.with_gripper = with_gripper
        self.sdk, self.clock = sdk, clock
        self.lock = threading.Lock()
        self.enabled = enabled
        self.pending = False
        self.request = None
        self.message = ('Arm and gripper enabled. Press M to release both.' if with_gripper
                        else 'Arm enabled. Press M to release all six joints.')
        self.latest = None
        self.sequence = 0

    def request_toggle(self):
        with self.lock:
            if self.pending:
                message, level, accepted = 'Motor switch ignored: another switch is in progress.', 'warning', False
            elif self.latest is None or self.clock() - self.latest[0] > .5:
                self.message = 'Fresh joint feedback is required before switching motor state.'
                message, level, accepted = self.message, 'warning', False
            else:
                # Enabled -> disable; unknown -> enable for explicit recovery.
                self.request = False if self.enabled is True else True
                self.pending = True
                self.message = ('Switching arm and gripper enable states...' if self.with_gripper
                                else 'Switching arm enable state...')
                target = self.request
                message, level, accepted = self.message, 'info', True
        if accepted:
            self.report('Motor switch requested: ' + ('enable.' if target else 'release.'))
        self.report(message, level)
        return accepted

    def apply_request(self):
        with self.lock:
            if not self.pending:
                return
            target = self.request
            cancelled = False
            if self.latest is None or self.clock() - self.latest[0] > .5:
                self.pending = False
                self.message = 'Motor switch cancelled: joint feedback is stale.'
                cancelled = True
                message = self.message
        if cancelled:
            self.report(message, 'warning')
            return
        level = 'info'
        try:
            set_drag_enabled(self.sdk, target, self.with_gripper)
        except Exception as exc:
            with self.lock:
                self.enabled = None
                self.message = str(exc) + '. Press M to request enable.'
                level = 'error'
        else:
            with self.lock:
                self.enabled = target
                self.message = ('Arm and gripper enabled. Press M to release both.' if target else
                                'Arm and gripper released. Drag by hand. Press M to lock both.')
                if not self.with_gripper:
                    self.message = ('Arm enabled. Press M to release all six joints.' if target else
                                    'Arm released. Drag by hand. Press M to lock all six joints.')
        finally:
            with self.lock:
                self.pending = False
                message = self.message
            self.report(message, level)

    def observe(self, angles, measured, wall):
        if (len(angles) != 6 or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                and math.isfinite(v) for v in angles)
                or not math.isfinite(measured) or not math.isfinite(wall)):
            return False
        with self.lock:
            if self.clock() - measured > .5 or (self.latest and measured <= self.latest[0]):
                return False
            self.sequence += 1
            self.latest = (measured, wall, tuple(math.radians(v) for v in angles), self.sequence)
            return True

    def snapshot(self):
        with self.lock:
            return self.enabled, self.pending, self.message, self.latest
