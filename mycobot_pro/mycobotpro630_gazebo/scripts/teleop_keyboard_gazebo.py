#!/usr/bin/env python3
"""Real-only Cartesian jog with collision-checked joint Init/Home moves."""
import math
import signal
import sys
import threading

import pro630_ros1 as ros
from pro630_ros1 import ExternalShutdownException
from pro630_ros1 import Node
from moveit_msgs.srv import GetStateValidity
from pro630_collision_check import CollisionCheck, DEFAULT_COLLISION_DEPTH_TOLERANCE_M
from pro630_real_keyboard import RealPoseReader
from pro630_model import KEYBOARD_ARM_FEEDBACK_MAX_AGE_S
from pro630_control_logging import ControlLog, add_position_limit_notice
from pro630_configuration import configure_node
from pro630_public_client import PublicPro630Client
from pro630_cartesian_keyboard import (
    CartesianKeyboardTransport, CARTESIAN_AXES, SPEED_GEARS, GRIPPER_SPEED_GEARS,
)

CARTESIAN_KEYS = {"w": (0, 1), "s": (0, -1), "a": (1, 1), "d": (1, -1),
                  "z": (2, -1), "x": (2, 1), "u": (3, 1), "j": (3, -1),
                  "i": (4, 1), "k": (4, -1), "o": (5, 1), "l": (5, -1)}


class TeleopKeyboard(CollisionCheck, Node):
    def __init__(self):
        super().__init__("teleop_keyboard_gazebo")
        configure_node(self)
        self.declare_parameter('mode', 'real')
        self.mode = str(self.get_parameter('mode').value).strip().lower()
        if self.mode != 'real':
            raise ValueError('Cartesian keyboard control requires the real environment')
        self.log = ControlLog(self.get_logger())
        self.mc = self.real_transport = self._real_thread = None
        self._closing = False
        self._pose_validation_cancelled = lambda: self._closing
        try:
            self.mc = PublicPro630Client(feature='keyboard', with_gripper=self.with_gripper, allow_collision=self.allow_collision)
            self.validity_client = self.create_client(
                GetStateValidity, '/check_state_validity', persistent=True)
            self.collision_depth_tolerance_m = DEFAULT_COLLISION_DEPTH_TOLERANCE_M
            self._joint_pose_reader = RealPoseReader(
                self.mc, self.joint_limits, with_gripper=self.with_gripper,
                report=self.get_logger().debug, arm_max_age=KEYBOARD_ARM_FEEDBACK_MAX_AGE_S)
            self.real_transport = CartesianKeyboardTransport(self.mc, self.with_gripper, report=self.log.event,
                                                              allow_collision=self.allow_collision,
                                                              pose_validator=self._validate_pose_path)
            self._real_thread = threading.Thread(target=self.real_transport.run, daemon=True)
            self._real_thread.start()
            self.log.event('Cartesian keyboard ready: base frame; controls locked. Press M to unlock.')
        except Exception:
            self.close()
            raise

    @property
    def _stop_requested(self):
        return self._closing or self._pose_validation_cancelled()

    def _validate_pose_path(self, target, cancelled):
        """Same sampled joint path/escape rules as slider and GUI joint moves."""
        self._pose_validation_cancelled = cancelled
        try:
            arm = [math.radians(value) for value in target]
            if any(not lo <= value <= hi for value, (lo, hi) in zip(arm, self.joint_limits[:6])):
                return False, 'Joint target exceeds its limits.'
            def read_pose():
                if self._stop_requested:
                    return None
                if self.with_gripper:
                    self._joint_pose_reader.force_gripper_read()
                pose = self._joint_pose_reader(moving=False)
                return None if self._stop_requested else pose
            current = read_pose()
            for _ in range(3):
                if current is None:
                    return False, 'STOP requested.'
                goal = arm + ([current[6]] if self.with_gripper else [])
                valid, reason = self._path_is_valid(current, goal)
                if not valid:
                    return False, reason
                confirmed = read_pose()
                if confirmed is None:
                    return False, 'STOP requested.'
                changed = max(abs(a-b) for a, b in zip(current[:6], confirmed[:6])) > .01
                if self.with_gripper:
                    changed = changed or abs(current[6] - confirmed[6]) > 1e-6
                if not changed:
                    return True, 'valid'
                current = confirmed
            return False, 'Start pose keeps changing; release external control and retry.'
        finally:
            self._pose_validation_cancelled = lambda: self._closing

    def arm_real(self):
        return self.real_transport.request_arm()

    def stop(self):
        self.real_transport.stop()
        self.log.event('STOP requested; real controls locked.')

    def set_hold_speed_gear(self, gear):
        self.real_transport.set_gear(gear)
        gear, speed = self.hold_speed_status()
        self.log.event(f'Gear {gear}: Cartesian SDK speed {speed}%; ' +
                       (f'gripper {GRIPPER_SPEED_GEARS[gear - 1]}.' if self.with_gripper else ''))

    def hold_speed_status(self):
        with self.real_transport.lock:
            return self.real_transport.gear, SPEED_GEARS[self.real_transport.gear - 1]

    def press_hold(self, axis, direction):
        return self.real_transport.request(axis, direction)

    def request_pose(self, name):
        return self.real_transport.request_pose(name)

    def save_home(self):
        return self.real_transport.request_save_home()

    def release_hold(self):
        self.real_transport.release()

    def hold_idle(self):
        return self.real_transport.idle()

    def hold_tick(self):
        self.real_transport.keepalive()

    def close(self):
        if self._closing:
            return
        self._closing = True
        try:
            if self.real_transport is not None:
                self.real_transport.stop()
                self.real_transport.shutdown.set()
            if self._real_thread is not None:
                self._real_thread.join(timeout=8.)
                if self._real_thread.is_alive():
                    self.log.event('Keyboard worker shutdown is not confirmed.', 'error')
            if self.real_transport is not None:
                for text, level in self.real_transport.drain_notices():
                    self.log.event(text, level)
        finally:
            if self.mc is not None:
                self.mc.close()
                self.mc = None



def hold_release_action(still_down):
    """Decide a KeyRelease from the physical key state, not the repeat rate.

    True means the key is still down, so an auto-repeat release is ignored.
    False means the key is up and the hold must stop. None means the state
    could not be read and the caller may use a short debounce.
    """
    if still_down is True:
        return "ignore"
    if still_down is False:
        return "release"
    return "debounce"


class X11KeyState:
    """Read the live X11 keymap. Auto-repeat does not clear a held key."""

    def __init__(self):
        self._lib = None
        self._display = None
        self._failed = False

    def down(self, keycode):
        import ctypes
        keycode = int(keycode or 0)
        if keycode <= 0 or keycode >= 256 or self._failed:
            return None
        if self._lib is None:
            try:
                self._lib = ctypes.CDLL("libX11.so.6")
                self._lib.XOpenDisplay.restype = ctypes.c_void_p
                self._lib.XQueryKeymap.argtypes = [
                    ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte)]
                self._display = self._lib.XOpenDisplay(None)
            except Exception:
                self._failed = True
                return None
            if not self._display:
                self._failed = True
                return None
        keys = (ctypes.c_ubyte * 32)()
        self._lib.XQueryKeymap(self._display, keys)
        return bool(keys[keycode // 8] & (1 << (keycode % 8)))


class CartesianHoldWindow:
    """Small Qt window that supplies real press/release events to the node."""

    def __init__(self, node):
        from python_qt_binding.QtCore import Qt, QTimer
        from python_qt_binding.QtWidgets import QLabel, QVBoxLayout, QHBoxLayout, QGroupBox, QWidget
        from python_qt_binding.QtGui import QFontDatabase

        class Window(QWidget):
            def __init__(self, owner):
                super().__init__()
                self.owner = owner

            def keyPressEvent(self, event):
                self.owner.key_press(event)

            def keyReleaseEvent(self, event):
                self.owner.key_release(event)

            def focusOutEvent(self, event):
                self.owner.log.event("Window focus lost; releasing held keys.")
                self.owner.release(source="focus lost")
                super().focusOutEvent(event)

            def closeEvent(self, event):
                self.owner.log.event("Keyboard window closed.")
                self.owner.release(source="window closed")
                self.owner.closing = True
                event.accept()

        self.node = node
        self.log = ControlLog(node.get_logger())
        self.Qt = Qt
        self.closing = False
        self.pressed_code = None
        self.pressed_scan = 0
        self.displayed_keys = {}
        self._observed_waiting_generation = 0
        self.key_state = X11KeyState()
        self.window = Window(self)
        self.window.setWindowTitle("Mycobot Pro630 Keyboard Control")
        self.window.setFocusPolicy(Qt.StrongFocus)
        layout = QVBoxLayout(self.window)
        layout.setSpacing(10)
        layout.addWidget(QLabel("Real | Base | Hold to move, release to stop"))
        fixed_font = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        motion_row = QHBoxLayout()
        for title, text in (
            ("Movement", "          W  X+\nA  Y+     S  X-     D  Y-\n          Z  Z-     X  Z+"),
            ("Rotation", "U  RX+   I  RY+   O  RZ+\nJ  RX-   K  RY-   L  RZ-"),
        ):
            group = QGroupBox(title)
            inner = QVBoxLayout(group)
            label = QLabel(text)
            label.setFont(fixed_font)
            inner.addWidget(label)
            motion_row.addWidget(group)
        layout.addLayout(motion_row)
        if node.with_gripper:
            group = QGroupBox("Gripper")
            inner = QVBoxLayout(group)
            inner.addWidget(QLabel("G  Open                         H  Close"))
            layout.addWidget(group)
        pose_group = QGroupBox("Pose")
        pose_layout = QVBoxLayout(pose_group)
        pose_layout.addWidget(QLabel("1  Go to init pose"))
        self.home_label = QLabel("2  Go to init home")
        pose_layout.addWidget(self.home_label)
        pose_layout.addWidget(QLabel("3  Save current pose as home"))
        layout.addWidget(pose_group)
        speed_group = QGroupBox("Speed")
        speed_layout = QHBoxLayout(speed_group)
        speed_layout.addWidget(QLabel("- / +"))
        self.gear_label = QLabel()
        self.speed_label = QLabel()
        speed_layout.addWidget(self.gear_label)
        speed_layout.addWidget(self.speed_label)
        layout.addWidget(speed_group)
        self._refresh_speed_display()
        control_group = QGroupBox("Control")
        control_layout = QVBoxLayout(control_group)
        control_layout.addWidget(QLabel("M  Unlock     N / Space  Stop     Q / Ctrl+C  Quit"))
        layout.addWidget(control_group)
        self.key_label = QLabel("Pressed key: None")
        layout.addWidget(self.key_label)
        self.status = QLabel("Locked. Press M.")
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(40)
        layout.addWidget(self.status)
        add_position_limit_notice(layout, node, self.log)
        self.log.event(self.status.text())
        self.window.resize(820, 470)
        self.timer = QTimer(self.window)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.tick)
        self.timer.start()
        self.release_timer = QTimer(self.window)
        self.release_timer.setSingleShot(True)
        self.release_timer.setInterval(70)
        self.release_timer.timeout.connect(self._release_if_key_up)

    def show(self):
        self.window.show()
        self.window.activateWindow()
        self.window.setFocus()

    def _set_status(self, text, level='info', state=None, log=True):
        self.status.setText(text)
        if not log:
            return
        if state is None:
            self.log.event(text, level)
        else:
            self.log.state('status', state, text, level)

    def release(self, clear_keys=True, source='control reset'):
        self.release_timer.stop()
        self.pressed_code = None
        self.pressed_scan = 0
        if clear_keys:
            for key in self.displayed_keys.values():
                self.log.event(f"Key {key} cleared ({source}; synthetic release).")
            self.displayed_keys.clear()
        self.node.release_hold()
        self._refresh_key_display()

    def _release_if_key_up(self):
        """Debounce fallback only. A key that is still down keeps moving."""
        if hold_release_action(self.key_state.down(self.pressed_scan)) == "ignore":
            self.release_timer.stop()
            return
        self.release(clear_keys=False)
        self._set_status("Key released: stop requested.")

    def request_exit(self, source):
        from python_qt_binding.QtWidgets import QApplication
        if self.closing:
            return
        self.log.event(f"{source}: exit requested.")
        self.closing = True
        self.release(source=source)
        self._set_status(f"{source}: braking, then exiting.")
        app = QApplication.instance()
        if app is not None:
            app.quit()

    def key_press(self, event):
        key = event.text().lower()
        ctrl_c = event.key() == self.Qt.Key_C and bool(event.modifiers() & self.Qt.ControlModifier)
        functional_key = (
            ctrl_c
            or
            key in CARTESIAN_KEYS
            or key in ("1", "2", "3", "+", "=", "-", "_")
            or event.key() in (self.Qt.Key_Space, self.Qt.Key_Q)
            or (self.node.with_gripper and key in ("g", "h"))
            or key in ("m", "n")
        )
        if functional_key and not event.isAutoRepeat():
            from python_qt_binding.QtGui import QKeySequence
            if event.key() not in self.displayed_keys:
                self.log.event(f"Key {QKeySequence(event.key()).toString() or event.text().upper()} pressed.")
            self.displayed_keys.pop(event.key(), None)
            self.displayed_keys[event.key()] = QKeySequence(event.key()).toString() or event.text().upper()
            self._refresh_key_display()
        if event.key() == self.pressed_code:
            # The held key's auto-repeat is another press, not a new coordinate axis.
            self.release_timer.stop()
            return
        if event.isAutoRepeat():
            return
        if ctrl_c:
            self.request_exit("Ctrl+C")
            return
        if key in ("m", "n"):
            if key == "m":
                result = self.node.arm_real()
                self._set_status(result)
            else:
                self.node.stop()
                # stop() already prints this exact response.
                self._set_status("STOP requested; real controls locked.", log=False)
            self._refresh_key_display()
            return
        if key in ("+", "=", "-", "_"):
            gear, _ = self.node.hold_speed_status()
            gear = max(1, min(5, gear + (1 if key in ("+", "=") else -1)))
            self._gear_changed(gear - 1)
            return
        if key in ("1", "2", "3"):
            result = self.node.save_home() if key == "3" else self.node.request_pose("Init" if key == "1" else "Home")
            if not result.startswith("rejected"):
                self.release_timer.stop()
                self.pressed_code = None
                self.pressed_scan = 0
            self._set_status(result, "warning" if result.startswith("rejected") else "info")
            self._refresh_key_display()
            return
        if event.key() == self.Qt.Key_Space:
            self.pressed_code = None
            self.node.stop()
            self._refresh_key_display()
            self._set_status("STOP requested (not a hardware E-stop).")
            return
        if event.key() == self.Qt.Key_Q:
            self.request_exit("Q")
            return
        if key in CARTESIAN_KEYS:
            axis, direction = CARTESIAN_KEYS[key]
        elif key == "g" and self.node.with_gripper:
            axis, direction = 6, 1
        elif key == "h" and self.node.with_gripper:
            axis, direction = 6, -1
        else:
            return
        self.release_timer.stop()
        result = self.node.press_hold(axis, direction)
        if not result.startswith(("rejected", "waiting")):
            # A rejected second key must not discard the active X key: its
            # eventual KeyRelease still has to stop an upward escape jog.
            self.pressed_code = event.key()
            self.pressed_scan = int(event.nativeScanCode() or 0)
        self._refresh_key_display()
        self._set_status(f"{CARTESIAN_AXES[axis]}: {result}.",
                         "warning" if result.startswith(("rejected", "waiting")) else "info")

    def key_release(self, event):
        if event.isAutoRepeat():
            return
        # Preserve the display across X11's synthetic auto-repeat releases.
        scan = int(event.nativeScanCode() or
                   (self.pressed_scan if event.key() == self.pressed_code else 0))
        action = hold_release_action(self.key_state.down(scan))
        if action != "ignore":
            label = self.displayed_keys.pop(event.key(), None)
            if label is not None:
                self.log.event(f"Key {label} released.")
            self._refresh_key_display()
        if event.key() != self.pressed_code:
            return
        if action == "ignore":
            # Linux auto-repeat emits KeyRelease while the key remains down.
            self.release_timer.stop()
            return
        if action == "release":
            self.release(clear_keys=False)
            self._set_status("Key released: stop requested.")
            return
        self.pressed_scan = scan
        self.release_timer.start()
        self._set_status("Key release detected: decelerating after debounce.")

    def _refresh_key_display(self):
        key = self.displayed_keys.get(self.pressed_code) if not self.node.hold_idle() else None
        if key is None:
            key = next(reversed(list(self.displayed_keys.values())), None)
        self.key_label.setText("Pressed key: " + (key or "None"))

    def _gear_changed(self, index):
        self.node.set_hold_speed_gear(int(index) + 1)
        self._refresh_speed_display()
        self._set_status(self.speed_label.text(), log=False)

    def _refresh_speed_display(self):
        gear, speed = self.node.hold_speed_status()
        self.gear_label.setText(f"Gear: {gear}")
        self.speed_label.setText(
            f"Cartesian: {speed}%" +
            (f"     Gripper: {GRIPPER_SPEED_GEARS[gear - 1]}" if self.node.with_gripper else ""))

    def tick(self):
        try:
            self._refresh_speed_display()
            self.node.hold_tick()
            transport = self.node.real_transport
            if transport.waiting_generation != self._observed_waiting_generation:
                self._observed_waiting_generation = transport.waiting_generation
                self.release(source="control fault")
            for text, level in transport.drain_notices():
                self._set_status(text, level)
            self._refresh_key_display()
        except Exception as exc:
            self.node.stop()
            self._set_status(f'Control error: {exc}', 'error', state=('control_error', str(exc)))


def main(args=None):
    from pro630_ros1 import SignalHandlerOptions
    from python_qt_binding.QtWidgets import QApplication
    ros.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    ros_thread = None
    exit_code = 0
    try:
        node = TeleopKeyboard()
        app = QApplication.instance() or QApplication(["pro630_cartesian_hold"])
        ros_thread = threading.Thread(target=ros.spin, args=(node,), daemon=True)
        ros_thread.start()
        window = CartesianHoldWindow(node)
        signal.signal(signal.SIGINT, lambda *_: window.request_exit('Ctrl+C'))
        signal.signal(signal.SIGTERM, lambda *_: window.request_exit('SIGTERM'))
        window.show()
        app.exec_()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        exit_code = 1
        print(f'Failed to start Pro630 Cartesian keyboard: {exc}', file=sys.stderr)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        except Exception as exc:
            exit_code = 1
            print(f'Cartesian keyboard cleanup failed: {exc}', file=sys.stderr)
        finally:
            try:
                ros.try_shutdown()
            finally:
                if ros_thread is not None:
                    ros_thread.join(timeout=1.)
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
