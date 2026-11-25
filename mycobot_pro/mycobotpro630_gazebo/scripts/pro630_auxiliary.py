#!/usr/bin/env python3
"""ROS 1 entry points for individual planning and visualization tools."""

from pathlib import Path
import shutil
import signal
import subprocess
import tempfile

import rosnode
import rospy
import yaml

from pro630_launch_config import PACKAGE, descriptions, move_group_params, read_yaml
from pro630_configuration import controllers, public_profile
from pro630_ros1 import package_directory
from pro630_session_support import stop_owned_children


def run_child(command):
    child = subprocess.Popen(command, start_new_session=True)
    try:
        return child.wait()
    finally:
        stop_owned_children([child])


def setup_assistant(root, with_gripper):
    # The Noetic editor writes robot_description while loading files. Keep
    # those editor parameters separate from the public environment.
    namespace = rospy.get_name()
    params = ['robot_description', 'robot_description_semantic']
    remaps = [f'/{name}:={namespace}/{name}' for name in params]
    command = ['rosrun', 'moveit_setup_assistant', 'moveit_setup_assistant']
    if rospy.get_param('~auto_load', False):
        command.append('--debug')
    with tempfile.TemporaryDirectory(prefix='pro630_setup_') as directory:
        config_root = Path(directory)
        config_dir = config_root / 'config'
        config_dir.mkdir()
        data = descriptions(root, with_gripper)
        (config_dir / 'firefighter.srdf').write_text(data['/robot_description_semantic'], encoding='utf-8')
        for filename in ('kinematics.yaml', 'ompl_planning.yaml', 'joint_limits.yaml'):
            shutil.copyfile(root / 'config' / filename, config_dir / filename)
        shutil.copyfile(root / 'package.xml', config_root / 'package.xml')
        settings = yaml.safe_load((root / '.setup_assistant').read_text(encoding='utf-8'))
        settings['moveit_setup_assistant_config']['URDF']['xacro_args'] = (
            'with_gripper:=' + str(with_gripper).lower())
        (config_root / '.setup_assistant').write_text(yaml.safe_dump(settings), encoding='utf-8')
        rospy.loginfo('Setup Assistant edits a temporary profile; select a destination to export changes.')
        try:
            return run_child(command + ['--config_pkg', str(config_root), *remaps])
        finally:
            for name in params:
                try:
                    rospy.delete_param(f'{namespace}/{name}')
                except KeyError:
                    pass


def run():
    root = Path(package_directory(PACKAGE))
    tool = rospy.get_param('~tool')
    option = rospy.get_param('~with_gripper', 'auto').lower()
    if option not in ('auto', 'true', 'false'):
        raise ValueError('with_gripper must be auto, true or false')
    active = '/pro630_environment_state' in rosnode.get_node_names()
    if active:
        with_gripper = public_profile()
        if option != 'auto' and (option == 'true') != with_gripper:
            raise RuntimeError('Installation profile does not match the public environment')
    else:
        with_gripper = option != 'false'
    if tool == 'spawn_controllers':
        if active:
            raise RuntimeError('The public environment owns controller startup')
        config = read_yaml(root, 'ros_controllers.yaml')
        for name in controllers(with_gripper):
            if not rospy.has_param('/' + name):
                rospy.set_param('/' + name, config[name])
        command = ['rosrun', 'controller_manager', 'spawner', *controllers(with_gripper)]
    elif tool == 'setup_assistant':
        return setup_assistant(root, with_gripper)
    else:
        if not active:
            for key, value in descriptions(root, with_gripper).items():
                rospy.set_param(key, value)
        if tool == 'moveit_rviz':
            command = ['rosrun', 'rviz', 'rviz', '-d', str(root / 'config/moveit.rviz'),
                       '__name:=pro630_rviz']
        elif tool == 'rsp':
            if active:
                raise RuntimeError('The public environment already owns robot_state_publisher')
            command = ['rosrun', 'robot_state_publisher', 'robot_state_publisher']
        elif tool == 'move_group':
            if active:
                raise RuntimeError('The public environment already owns move_group')
            for key, value in move_group_params(root, with_gripper).items():
                rospy.set_param(key, value)
            command = ['roslaunch', PACKAGE, '_move_group.launch', 'environment:=simulation']
        else:
            raise ValueError(f'Unknown auxiliary tool: {tool}')
    return run_child(command)


if __name__ == '__main__':
    rospy.init_node('pro630_auxiliary', disable_signals=True)
    signal.signal(signal.SIGTERM, signal.getsignal(signal.SIGINT))
    try:
        raise SystemExit(run())
    except KeyboardInterrupt:
        pass
