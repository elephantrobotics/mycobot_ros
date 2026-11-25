#!/usr/bin/env python3
"""Real-only combined arm/gripper enable switching and measured pose mirroring."""
import math
import signal
import sys
import threading
import time

import pro630_ros1 as ros
from pro630_ros1 import Node

from pro630_drag_support import DragState, set_drag_enabled
from pro630_public_client import PublicPro630Client as Pro630Client
from pro630_session_support import read_initial_gripper
from pro630_configuration import configure_node
from pro630_model import DEFAULT_PRO630_IP, DEFAULT_PRO630_PORT
from pro630_control_logging import ControlLog, add_position_limit_notice

class DragFollow(Node):
    def __init__(self):
        super().__init__('pro630_drag_follow')
        self.log = ControlLog(self.get_logger())
        configure_node(self)
        self.declare_parameter('startup_enabled', False)
        self.declare_parameter('initial_gripper', -1.0)
        self.sdk = None
        self.worker = None
        self.stop_event = threading.Event()
        self._closing = False
        self._exit_mode_required = False
        try:
            if {'slider_control_gazebo', 'teleop_keyboard_gazebo'} & set(self.get_node_names()):
                raise RuntimeError('Close the existing robot controller before starting drag following')
            self.sdk = Pro630Client(DEFAULT_PRO630_IP, DEFAULT_PRO630_PORT, feedback_hz=20.0, ownership_timeout=0, feature="drag", with_gripper=self.with_gripper, allow_collision=self.allow_collision)
            if not self.get_parameter('startup_enabled').value:
                if self.sdk.is_moving() != 0:
                    raise RuntimeError('Stop robot motion before starting drag following')
                self._exit_mode_required = True
                self.log.event('Startup motor enable requested.')
                set_drag_enabled(self.sdk, True, self.with_gripper)
                self.log.event('Startup motor enable acknowledged.')
            else:
                self._exit_mode_required = True
                self.log.event('Startup motor enable was completed by the control session.')
            if self.with_gripper:
                self.gripper = float(self.get_parameter('initial_gripper').value)
                if not math.isfinite(self.gripper) or not 0 <= self.gripper <= 1:
                    self.gripper = read_initial_gripper(self.sdk) / 100.0
            self.state = DragState(self.sdk, with_gripper=self.with_gripper, report=self.log.event)
            self.log.event(self.state.message)
            self.sdk.set_angle_callback(self._sample)
            self.worker = threading.Thread(target=self._read_loop, daemon=True)
            self.worker.start()
        except Exception:
            self.close()
            raise

    def _sample(self, sample):
        self.state.observe(sample.angles, sample.monotonic, sample.wall_time)

    def _read_loop(self):
        # One SDK caller: motor switches and angle queries cannot race or build a queue.
        while not self.stop_event.is_set():
            try:
                self.state.apply_request()
                if self.stop_event.is_set():
                    break
                angles = self.sdk.get_angles()
                stamp = self.sdk.sample_time('arm')
                if stamp and isinstance(angles, (list, tuple)):
                    self.state.observe(angles, *stamp)
            except Exception:
                # The UI reports stale feedback; retry queries without terminal spam.
                self.stop_event.wait(.05)


    def render(self):
        enabled, pending, message, sample = self.state.snapshot()
        if sample is None or time.monotonic() - sample[0] > .5:
            return enabled, pending, message, False
        return enabled, pending, message, True

    def close(self):
        if self._closing:
            return
        self._closing = True
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=5.0)
        if self.sdk is not None:
            try:
                if self._exit_mode_required:
                    if self.worker is not None and self.worker.is_alive():
                        self.log.event('Exit free-move request not confirmed: control worker is still active.', 'error')
                    else:
                        self.log.event(
                            'Exit requested: disabling arm free-move mode and locking the gripper.'
                            if self.with_gripper else
                            'Exit requested: disabling arm free-move mode.')
                        # Same order as M: leave six-axis free-move, then lock the gripper.
                        set_drag_enabled(self.sdk, True, self.with_gripper)
                        self.log.event(
                            'Arm free-move exit and gripper lock acknowledged.'
                            if self.with_gripper else
                            'Arm free-move exit acknowledged.')
            except Exception as exc:
                self.log.event(f'Exit free-move request failed: {exc}', 'error')
            finally:
                self.sdk.close()
                self.sdk = None
        self.log.state('closing', True, 'Drag following closed.')


def main():
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
    from pro630_ros1 import SignalHandlerOptions
    ros.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    app = QApplication(sys.argv[:1])

    class Window(QWidget):
        def __init__(self):
            super().__init__()
            self.held_keys = set()
            self.setWindowTitle('Pro630 Drag Follow Control')
            self.setFocusPolicy(Qt.StrongFocus)
            font = self.font()
            font.setPointSize(11)
            self.setFont(font)
            layout = QVBoxLayout(self)
            layout.setContentsMargins(14, 12, 14, 12)
            layout.setSpacing(8)
            profile = 'Arm + gripper' if node.with_gripper else 'Arm only'
            layout.addWidget(QLabel(f'Real | {profile}'))
            self.mode_label = QLabel()
            layout.addWidget(self.mode_label)
            layout.addWidget(QLabel('M  Release / Lock     Q  Quit'))
            self.key_label = QLabel('Pressed key: None')
            layout.addWidget(self.key_label)
            self.status_label = QLabel()
            self.status_label.setWordWrap(True)
            layout.addWidget(self.status_label)
            self.feedback_label = QLabel()
            layout.addWidget(self.feedback_label)
            add_position_limit_notice(layout, node, node.log)
            closing = ('Closing exits free-move mode and locks the gripper.'
                       if node.with_gripper else
                       'Closing exits arm free-move mode.')
            layout.addWidget(QLabel('Support the arm while released. ' + closing))
            self.update_status(*node.render())
            self.resize(620, 220)

        def update_status(self, enabled, pending, message, fresh):
            if pending:
                title = 'Switching...'
            elif enabled is True:
                title = 'Locked'
            elif enabled is False:
                title = 'Released (free move)'
            else:
                title = 'Not confirmed'
            self.mode_label.setText('State: ' + title)
            normal_messages = (
                'Arm and gripper enabled. Press M to release both.',
                'Arm and gripper released. Drag by hand. Press M to lock both.',
                'Arm enabled. Press M to release all six joints.',
                'Arm released. Drag by hand. Press M to lock all six joints.',
            )
            normal = not pending and type(enabled) is bool and message in normal_messages
            self.status_label.setText('' if normal else message)
            self.status_label.setVisible(not normal)
            self.feedback_label.setText('Feedback: live' if fresh else 'Feedback: unavailable')

        def _refresh_keys(self):
            names = [('M' if key == Qt.Key_M else 'Q') for key in sorted(self.held_keys)]
            self.key_label.setText('Pressed key: ' + (' + '.join(names) if names else 'None'))

        def keyPressEvent(self, event):
            if event.isAutoRepeat():
                return
            if event.key() == Qt.Key_M:
                self._pressed(event.key(), 'M')
                node.state.request_toggle()
            elif event.key() == Qt.Key_Q:
                self._pressed(event.key(), 'Q')
                self.close()
            else:
                super().keyPressEvent(event)

        def _pressed(self, key, label):
            if key not in self.held_keys:
                self.held_keys.add(key)
                node.log.event(f'Key {label} pressed.')
                self._refresh_keys()

        def keyReleaseEvent(self, event):
            if not event.isAutoRepeat() and event.key() in self.held_keys:
                self.held_keys.remove(event.key())
                label = 'M' if event.key() == Qt.Key_M else 'Q'
                node.log.event(f'Key {label} released.')
                self._refresh_keys()
            super().keyReleaseEvent(event)

        def _clear_keys(self, source):
            for key in self.held_keys:
                node.log.event(f'Key {"M" if key == Qt.Key_M else "Q"} cleared ({source}; synthetic release).')
            self.held_keys.clear()
            self._refresh_keys()

        def focusOutEvent(self, event):
            self._clear_keys('focus lost')
            super().focusOutEvent(event)

        def closeEvent(self, event):
            node.log.event(
                'Drag follow window closed; arm free-move exit and gripper lock will be requested.'
                if node.with_gripper else
                'Drag follow window closed; arm free-move exit will be requested.')
            self._clear_keys('window closed')
            super().closeEvent(event)

    try:
        node = DragFollow()
        window = Window()
        timer = QTimer(window)

        def tick():
            if not ros.ok():
                window.close()
                return
            ros.spin_once(node, timeout_sec=0)
            enabled, pending, message, fresh = node.render()
            node.log.state('feedback', fresh, 'Live joint feedback.' if fresh else
                           'Joint feedback unavailable; Gazebo holds the last valid pose.',
                           'info' if fresh else 'warning')
            window.update_status(enabled, pending, message, fresh)

        timer.timeout.connect(tick)
        timer.start(20)
        def interrupted(source):
            node.log.event(f'{source}: exit requested.')
            window.close()
        signal.signal(signal.SIGINT, lambda *_: interrupted('Ctrl+C'))
        signal.signal(signal.SIGTERM, lambda *_: interrupted('SIGTERM'))
        window.show()
        return app.exec_()
    except Exception as exc:
        print(f'Failed to start drag following: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        ros.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
