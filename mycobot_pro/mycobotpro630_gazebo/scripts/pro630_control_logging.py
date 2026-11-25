"""Operation logs and transition-only status logs; never queries robot state."""
import sys
import threading

POSITION_LIMIT_TOPIC = '/pro630/position_limit_exceeded'
POSITION_LIMIT_MESSAGE = 'Position limit exceeded'
POSITION_LIMIT_SHUTDOWN_S = 2.0


def add_position_limit_notice(layout, node, log):
    from PyQt5.QtCore import Qt, pyqtSignal
    from PyQt5.QtWidgets import QLabel
    from std_msgs.msg import Bool

    class Notice(QLabel):
        visibility_requested = pyqtSignal(bool)

        def __init__(self):
            super().__init__(POSITION_LIMIT_MESSAGE)
            self.hide()
            self.visibility_requested.connect(self.setVisible, Qt.QueuedConnection)

        def receive(self, message):
            if message.data:
                log.state('position_limit', True, POSITION_LIMIT_MESSAGE, 'error')
            else:
                log.clear('position_limit')
            self.visibility_requested.emit(message.data)

    notice = Notice()
    layout.addWidget(notice)
    node.create_subscription(Bool, POSITION_LIMIT_TOPIC, notice.receive, 10)


def emit_log(logger, text, level='info'):
    """Keep Humble call-site severity fixed and logging out of control failures."""
    try:
        # Humble caches severity by source call site. Never dynamically dispatch
        # different logger methods from a single expression here.
        if level == 'info':
            logger.info(text)
        elif level == 'warning':
            logger.warning(text)
        elif level == 'error':
            logger.error(text)
        elif level == 'debug':
            logger.debug(text)
        else:
            raise ValueError(f'Unsupported log level: {level}')
        return True
    except Exception as exc:
        # Reporting must never prevent a rejection, STOP, or cleanup. Even the
        # fallback can fail when launch has already closed its output pipe.
        try:
            print(f'Logging failed ({exc}): {text}', file=sys.stderr)
        except Exception:
            pass
        return False


class ControlLog:
    def __init__(self, logger):
        self.logger = logger
        self._states = {}
        self._lock = threading.Lock()

    def event(self, text, level='info'):
        return emit_log(self.logger, text, level)

    def state(self, channel, key, text, level='info'):
        # Keys describe the condition, not changing feedback ages/readouts.
        with self._lock:
            if channel in self._states and self._states[channel] == key:
                return
            self._states[channel] = key
        self.event(text, level)

    def clear(self, channel):
        with self._lock:
            self._states.pop(channel, None)


def command_details(kind, target, speed, *, joint_frame=None):
    units = {'joints': 'deg', 'coords': 'mm/deg', 'gripper': 'normalized 0..1',
             'Execute': 'rad; gripper normalized 0..1',
             'Force Execute': 'rad; gripper normalized 0..1',
             'Random target loaded': 'rad; gripper normalized 0..1',
             'Model zero target loaded': 'rad; gripper normalized 0..1'}
    unit = units.get(kind, 'none')
    if joint_frame is not None:
        unit = f'{joint_frame}; {unit}'
    return f'{kind} requested: target={target} ({unit}), speed={speed}%.'


def track_speed(spin, log):
    """Observe committed edits/steps without changing spin-box value semantics."""
    from PyQt5.QtCore import QEvent, QObject, QTimer, Qt

    class SpeedObserver(QObject):
        def __init__(self):
            super().__init__(spin)
            self.last_value = spin.value()

        def record(self):
            value = spin.value()
            if value != self.last_value:
                self.last_value = value
                log.event(f'Speed set to {value}%.')

        def eventFilter(self, watched, event):
            if (event.type() in (QEvent.MouseButtonRelease, QEvent.Wheel)
                    or (event.type() == QEvent.KeyRelease and event.key() in
                        (Qt.Key_Up, Qt.Key_Down, Qt.Key_PageUp, Qt.Key_PageDown))):
                QTimer.singleShot(0, self.record)
            return False

    observer = SpeedObserver()
    spin._control_log_speed_observer = observer
    spin.editingFinished.connect(observer.record)
    spin.installEventFilter(observer)
    spin.lineEdit().installEventFilter(observer)
