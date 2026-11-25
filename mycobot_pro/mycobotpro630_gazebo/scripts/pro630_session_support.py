"""Launch-session contracts, independent of Qt, ROS and motion controllers."""
import math
import os
import signal
import subprocess
import threading
import time

from pro630_configuration import ARM_JOINTS, LIMITS_DEG, joints, require_flag, collision_policy
JOINTS = joints(True)


class EnvironmentLatch:
    def __init__(self, environment, clock=time.monotonic):
        self.environment = environment
        self.clock = clock
        self.state = None
        self.received = None
        self.session_id = None

    def observe(self, state):
        if state.get('environment') != self.environment:
            raise RuntimeError('Control and environment modes do not match')
        require_flag(state.get('with_gripper'))
        policy = collision_policy(state.get('allow_collision', False))
        if policy and self.environment != 'real':
            raise ValueError('allow_collision is supported only by the real environment')
        if self.state is not None and policy != self.state.get('allow_collision', False):
            raise RuntimeError('Collision policy changed; restart the control launch')
        if self.state is not None and state['with_gripper'] != self.state['with_gripper']:
            raise RuntimeError('Installation profile changed; restart the control launch')
        session = state.get('session_id')
        if not isinstance(session, str) or not session:
            raise ValueError('Missing environment session ID')
        if self.session_id is not None and session != self.session_id:
            raise RuntimeError('Environment restarted; restart the control launch')
        self.session_id = session
        self.state, self.received = state, self.clock()

    def fresh(self):
        return self.received is not None and self.clock() - self.received < 2.0

    def ready(self):
        return self.fresh() and self.state.get('ready') is True


def read_initial_snapshot(sdk, clock=time.monotonic, sleep=time.sleep, with_gripper=True):
    """Only query hardware. Keep the existing five-sample startup criteria."""
    if sdk.is_power_on() != 1:
        raise RuntimeError('Pro630 is not powered; automatic power-on is forbidden')
    error = sdk.get_error_information()
    if error not in (0, None):
        raise RuntimeError(f'Pro630 reports error {error}')
    samples = []
    for index in range(5):
        if sdk.is_moving() != 0:
            raise RuntimeError('Pro630 must be stationary for the initial snapshot')
        angles = sdk.get_angles()
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            raise RuntimeError('Incomplete initial joint feedback')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) or not lo <= v <= hi
               for v, (lo, hi) in zip(angles, LIMITS_DEG)):
            raise RuntimeError('Invalid initial joint feedback')
        samples.append(list(angles))
        if index < 4:
            sleep(0.2)
    if any(max(p[i] for p in samples) - min(p[i] for p in samples) > 0.2
           for i in range(6)):
        raise RuntimeError('Pro630 initial pose is not stable')
    gripper = read_initial_gripper(sdk, clock, sleep) if with_gripper else None
    # Confirm the arm stayed still during a possibly slow gripper read.
    if sdk.is_moving() != 0:
        raise RuntimeError('Pro630 moved during the initial snapshot')
    final = sdk.get_angles()
    if (not isinstance(final, (list, tuple)) or len(final) != 6 or
            any(not isinstance(v, (int, float)) or isinstance(v, bool)
                or not math.isfinite(v) or not lo <= v <= hi or abs(v - old) > 0.2
                for v, old, (lo, hi) in zip(final, samples[-1], LIMITS_DEG))):
        raise RuntimeError('Pro630 pose changed during the initial snapshot')
    return [math.radians(v) for v in final] + ([gripper / 100.0] if with_gripper else [])


def read_initial_gripper(sdk, clock=time.monotonic, sleep=time.sleep, timeout=10.0):
    deadline = clock() + timeout
    finished, cancelled = threading.Event(), threading.Event()
    result = {'first': None, 'last': None}
    def read():
        try:
            while not cancelled.is_set() and clock() < deadline:
                try:
                    value = sdk.get_pro_gripper_angle()
                except Exception as exc:
                    value = str(exc)
                    transport_error = getattr(sdk, 'get_transport_error', None)
                    if callable(transport_error) and transport_error():
                        result['error'] = transport_error()
                        return  # A permanently closed stream cannot recover by retrying.
                if result['first'] is None:
                    result['first'] = value
                result['last'] = value
                if (not isinstance(value, bool) and isinstance(value, (int, float))
                        and math.isfinite(value) and 0 <= value <= 100
                        and not cancelled.is_set() and clock() < deadline):
                    result['value'] = value
                    return
                sleep(min(0.1, max(0.0, deadline - clock())))
        finally:
            finished.set()
    worker = threading.Thread(target=read, daemon=True)
    worker.start()
    completed = finished.wait(timeout)
    if 'error' in result:
        cancelled.set()
        raise RuntimeError(f'Initial gripper feedback failed: {result["error"]}')
    if not completed or 'value' not in result:
        cancelled.set()
        raise RuntimeError(f'Initial gripper feedback timed out after {timeout:g} s '
                           f'(first={result["first"]!r}, last={result["last"]!r})')
    worker.join()
    return result['value']


def control_commands(feature, environment, executable_directory, python, show_force_execute=False, with_gripper=True, allow_collision=False):
    """Explicit process allowlist: no Gazebo, MoveIt or additional SDK owner."""
    from pathlib import Path
    root = Path(executable_directory)
    policy = collision_policy(allow_collision)
    if policy and environment != 'real':
        raise ValueError('allow_collision is supported only by the real environment')
    def command(script, parameters):
        parameters = dict(parameters, with_gripper='true' if require_flag(with_gripper) else 'false',
                          allow_collision='true' if policy else 'false')
        args = [python, '-B', str(root / script)]
        for name, value in parameters.items():
            args.append(f'_{name}:={value}')
        return args
    if feature == 'slider':
        return [command('slider_control_gazebo.py', {
            'mode': environment, 'use_sim_time': 'true'}),
            command('pro630_slider_gui.py', {
                'environment': environment,
                'show_force_execute': 'true' if show_force_execute else 'false',
                'use_sim_time': 'false' if environment == 'real' else 'true'})]
    if feature == 'gui':
        if environment != 'real':
            raise ValueError('GUI control requires the real environment')
        return [command('pro630_gui_backend.py', {'use_sim_time': 'false'}),
                command('pro630_control_gui.py', {'use_sim_time': 'false'})]
    if feature == 'keyboard':
        if environment != 'real':
            raise ValueError('Cartesian keyboard control requires the real environment')
        return [command('teleop_keyboard_gazebo.py', {
            'mode': 'real', 'use_sim_time': 'false'})]
    if feature == 'drag':
        if environment != 'real':
            raise ValueError('Drag following requires the real environment')
        return [command('pro630_drag_follow.py', {'use_sim_time': 'false'})]
    raise ValueError('Unsupported control feature')


def stop_owned_children(children, send_signal=None, clock=time.monotonic):
    """Stop all owned process groups together within a shared eight-second budget."""
    send_signal = os.killpg if send_signal is None else send_signal
    remaining = [child for child in children if child.poll() is None]
    for sig, grace in ((signal.SIGINT, 5.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
        for child in remaining:
            try:
                send_signal(child.pid, sig)
            except ProcessLookupError:
                pass  # Child exited between poll and signal; still reap it.
        deadline = clock() + grace
        pending = []
        for child in remaining:
            try:
                child.wait(timeout=max(0.0, deadline - clock()))
            except subprocess.TimeoutExpired:
                pending.append(child)
        remaining = pending
        if not remaining:
            return
    raise RuntimeError('An owned control process did not exit after shutdown signals')
