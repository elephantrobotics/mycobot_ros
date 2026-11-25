#!/usr/bin/env python3
"""ROS 1 public environment and ordered Gazebo startup."""
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import uuid

import rospy
import yaml
from controller_manager_msgs.srv import LoadController, SwitchController, SwitchControllerRequest
from std_srvs.srv import Empty

from pro630_configuration import controllers, joints
from pro630_launch_config import PACKAGE, descriptions, move_group_params, profile, read_yaml
from pro630_model import DEFAULT_PRO630_IP, DEFAULT_PRO630_PORT
from pro630_ros1 import ServiceClient, get_logger, master_key, package_directory
from pro630_sdk_adapter import RobotConnectionLease
from pro630_session_support import stop_owned_children

MULTICAST_ERROR = re.compile(
    r'Exception sending a multicast message:[ \t]*|No such device|Network is unreachable'
    r'|没有那个设备|没有这个设备|网络不可达')
GAZEBO_SESSION_ENV = 'PRO630_GAZEBO_SESSION'


def gazebo_session_processes(session):
    """Find only this launch's Gazebo binaries and their ROS shell wrappers."""
    marker = f'{GAZEBO_SESSION_ENV}={session}'.encode()
    processes = []
    for path in Path('/proc').iterdir():
        if not path.name.isdecimal():
            continue
        try:
            argv = (path / 'cmdline').read_bytes().split(b'\0')
            executable = os.path.basename(argv[0])
            if executable in (b'sh', b'bash', b'dash') and len(argv) > 1:
                executable = os.path.basename(argv[1])
            if (executable in (b'gzserver', b'gzclient')
                    and marker in (path / 'environ').read_bytes().split(b'\0')):
                processes.append(int(path.name))
        except OSError:
            continue  # An unrelated process is unreadable or has already exited.
    return processes


def stop_gazebo_session(session):
    remaining = gazebo_session_processes(session)
    if not remaining:
        return
    rospy.logwarn(f'Cleaning up remaining Gazebo processes from this launch: {remaining}')
    for sig, grace in ((signal.SIGTERM, 1.0), (signal.SIGKILL, .5)):
        for pid in remaining:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            remaining = gazebo_session_processes(session)
            if not remaining:
                return
            time.sleep(.05)
        remaining = gazebo_session_processes(session)
        if not remaining:
            return
    raise RuntimeError(f'Gazebo processes still running after cleanup: {remaining}')


def stream_gazebo_output(stream):
    with stream:
        for line in stream:
            if MULTICAST_ERROR.search(line):
                remainder = MULTICAST_ERROR.sub('', line).strip()
                line = 'Network device unavailable: Gazebo multicast discovery could not send data.'
                if remainder:
                    line += ' ' + remainder
                line += '\n'
            print(line, end='', flush=True)


class Environment:
    def __init__(self):
        self.root = Path(package_directory(PACKAGE))
        self.directory = Path(__file__).resolve().parent
        self.mode = rospy.get_param('~environment', 'simulation')
        self.with_gripper = rospy.get_param('~with_gripper', False)
        self.allow_collision = rospy.get_param('~allow_collision', False)
        if self.mode not in ('real', 'simulation'):
            raise ValueError('environment must be real or simulation')
        if any(type(value) is not bool for value in (self.with_gripper, self.allow_collision)):
            raise ValueError('with_gripper and allow_collision must be booleans')
        if self.allow_collision and self.mode != 'real':
            raise ValueError('allow_collision is supported only with environment:=real')
        self.session = uuid.uuid4().hex
        self.children = []
        self.parameters = {}

    def start(self, command, persistent=True, gazebo_output=False):
        output = (dict(stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors='replace',
                       env=dict(os.environ, **{GAZEBO_SESSION_ENV: self.session}))
                  if gazebo_output else {})
        proc = subprocess.Popen(command, start_new_session=True, **output)
        if persistent:
            self.children.append((command[0:3], proc))
        if gazebo_output:
            threading.Thread(target=stream_gazebo_output, args=(proc.stdout,), daemon=True).start()
        return proc

    def start_node(self, script, params=None, *, flags=(), persistent=True):
        args = [sys.executable, '-B', str(self.directory / script), *flags]
        for name, value in (params or {}).items():
            args.append(f'_{name}:={str(value).lower() if type(value) is bool else value}')
        return self.start(args, persistent)

    def set_params(self, values):
        for name, value in values.items():
            if name not in self.parameters:
                self.parameters[name] = (rospy.has_param(name), rospy.get_param(name, None), value)
            rospy.set_param(name, value)

    def alive(self):
        if rospy.is_shutdown():
            raise KeyboardInterrupt
        for command, proc in self.children:
            code = proc.poll()
            if code is not None:
                raise RuntimeError(f'Public environment process exited ({code}): {command}')

    def complete(self, proc, title, timeout):
        deadline = time.monotonic() + timeout
        try:
            while proc.poll() is None and time.monotonic() < deadline:
                self.alive()
                time.sleep(.05)
            if proc.poll() != 0:
                raise RuntimeError(f'{title} failed (exit={proc.poll()})')
        finally:
            stop_owned_children([proc])

    def call(self, kind, name, request=None, timeout=10.):
        client = ServiceClient(kind, name)
        deadline = time.monotonic() + timeout
        while not client.service_is_ready() and time.monotonic() < deadline:
            self.alive()
            time.sleep(.05)
        if not client.service_is_ready():
            raise RuntimeError(f'Service unavailable: {name}')
        future = client.call_async(request if request is not None else kind._request_class())
        while not future.done() and time.monotonic() < deadline:
            self.alive()
            time.sleep(.05)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'Service call timed out: {name}')
        return future.result()

    def run(self):
        real = self.mode == 'real'
        common = {'environment': self.mode, 'session_id': self.session, 'with_gripper': self.with_gripper,
                  'allow_collision': self.allow_collision}
        self.set_params(descriptions(self.root, self.with_gripper))
        self.start_node('pro630_environment_state.py', common)
        snapshot = self.root / 'config/initial_positions.yaml'
        if real:
            snapshot = Path(rospy.get_param('~real_snapshot_file',
                                           '/tmp/mycobotpro630_real_initial_positions.yaml'))
            timeout = float(rospy.get_param('~real_snapshot_timeout_sec', 600.))
            self.start_node('pro630_robot_bridge.py', {
                'with_gripper': self.with_gripper, 'allow_collision': self.allow_collision,
                'pro630_ip': rospy.get_param('~pro630_ip', DEFAULT_PRO630_IP),
                'pro630_port': rospy.get_param('~pro630_port', DEFAULT_PRO630_PORT),
            })
            self.set_params({'/pro630_pose_gate/robot_description': rospy.get_param('/robot_description')})
            gate = self.start_node('pro630_pose_gate.py', {
                'with_gripper': self.with_gripper, 'allow_collision': self.allow_collision,
                'output_file': str(snapshot), 'timeout_sec': timeout}, persistent=False)
            self.complete(gate, 'Stable real Pro630 pose acquisition', timeout + 5.)

        data = yaml.safe_load(snapshot.read_text(encoding='utf-8'))['initial_positions']
        config = read_yaml(self.root, 'ros_controllers.yaml')
        if not self.with_gripper:
            del config['pro_gripper_controller']
        self.set_params({
            '/use_sim_time': True,
            '/pro630/environment': self.mode,
            '/pro630/initial_positions': {name: float(data[name]) for name in joints(self.with_gripper)},
            '/gazebo_spawn/robot_description': profile(self.root, self.with_gripper, gazebo=True, contact=not real),
            **{f'/{key}': value for key, value in config.items()},
        })
        self.start(['rosrun', 'robot_state_publisher', 'robot_state_publisher', '__name:=robot_state_publisher'])
        gui = str(rospy.get_param('~gazebo_gui', True)).lower()
        self.start(['roslaunch', 'gazebo_ros', 'empty_world.launch', '--screen',
                    '--sigint-timeout', '2', '--sigterm-timeout', '1',
                    'paused:=true', f'gui:={gui}'], gazebo_output=True)
        spawn = self.start(['rosrun', 'gazebo_ros', 'spawn_model', '-urdf', '-param',
                            '/gazebo_spawn/robot_description', '-model', 'mycobotpro630'], persistent=False)
        self.complete(spawn, 'Gazebo model spawn', 45.)
        names = controllers(self.with_gripper)
        for name in names:
            if not self.call(LoadController, '/controller_manager/load_controller',
                             LoadController._request_class(name=name), timeout=30.).ok:
                raise RuntimeError(f'Could not load controller {name}')
        self.call(Empty, '/gazebo/unpause_physics')
        request = SwitchControllerRequest(start_controllers=names, strictness=SwitchControllerRequest.STRICT)
        request.timeout = 10.
        if not self.call(SwitchController, '/controller_manager/switch_controller', request, timeout=15.).ok:
            raise RuntimeError('Could not start Pro630 controllers as a group')
        if real:
            tolerance = rospy.get_param('~initial_pose_tolerance_rad', .01)
            timeout = float(rospy.get_param('~initial_pose_timeout_sec', 20.))
            verifier = self.start_node('verify_initial_pose.py', {
                'expected_file': str(snapshot), 'with_gripper': self.with_gripper,
                'tolerance_rad': tolerance, 'timeout_sec': timeout}, persistent=False)
            self.complete(verifier, 'Gazebo initial pose verification', timeout + 5.)
        self.set_params(move_group_params(self.root, self.with_gripper))
        self.start(['roslaunch', PACKAGE, '_move_group.launch', f'environment:={self.mode}'])
        self.start_node('pro630_collision_scene.py')
        announce = self.start_node('pro630_environment_state.py', common,
                                   flags=('--announce-ready',), persistent=False)
        self.complete(announce, 'Environment acknowledgement', 65.)
        while not rospy.is_shutdown():
            self.alive()
            time.sleep(.1)

    def close(self):
        try:
            try:
                stop_owned_children([proc for _, proc in self.children])
            finally:
                stop_gazebo_session(self.session)
        finally:
            for name, (existed, old, value) in reversed(list(self.parameters.items())):
                try:
                    if rospy.get_param(name, None) != value:
                        continue
                    if existed:
                        rospy.set_param(name, old)
                    else:
                        rospy.delete_param(name)
                except (KeyError, OSError):
                    pass


def main():
    rospy.init_node('pro630_bringup', disable_signals=True)
    signal.signal(signal.SIGTERM, signal.getsignal(signal.SIGINT))
    logger = get_logger()
    environment = lease = None
    result = 0
    try:
        lease = RobotConnectionLease(f'pro630-ros1-environment-{master_key()}', 0, timeout=0)
        environment = Environment()
        environment.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        logger.error(f'Pro630 public environment failed: {exc}')
        result = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            if environment is not None:
                environment.close()
        finally:
            if lease is not None:
                lease.close()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
