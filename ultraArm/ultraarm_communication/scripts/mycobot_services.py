#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mycobot_services.py
This ROS node provides service interfaces for controlling a ultraArm P1 robotic arm.
It includes services to:
    - Set/Get joint angles
    - Set/Get end-effector coordinates
    - Switch gripper status

The node ensures serial port safety using file locking to prevent conflicts
when multiple processes access the robot simultaneously.

Author: WangWeiJian
Date: 2025-11-24
"""

import threading
import time
import rospy
import os
import fcntl
from ultraarm_communication.srv import *

import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '4.0.5'

current_verison = pymycobot.__version__
rospy.loginfo('Current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'The current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
else:
    rospy.loginfo('pymycobot library version meets the requirements!')
    from pymycobot import UltraArmP1
    from pymycobot.robot_info import RobotLimit

mc = None
latest_angles = [0.0, 0.0, 90.0, 0.0]
latest_coords = [0.0, 0.0, 0.0, 0.0]

ROBOT_LIMIT = RobotLimit.robot_limit.get("UltraArmP1", {})


def format_limit_value(value):
    """Format positive limits with a leading plus sign."""
    return f"+{value}" if value >= 0 else str(value)

def acquire(lock_file: str) -> int:
    """Acquire a file lock to prevent serial port conflicts.

    Args:
        lock_file (str): Path to the lock file.

    Returns:
        int: File descriptor if lock is acquired, None otherwise.
    """
    open_mode = os.O_RDWR | os.O_CREAT | os.O_TRUNC
    fd = os.open(lock_file, open_mode)

    pid = os.getpid()
    lock_file_fd = None
    
    timeout = 50.0
    start_time = current_time = time.time()
    while current_time < start_time + timeout:
        try:
            # LOCK_EX: exclusive lock, LOCK_NB: non-blocking
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (IOError, OSError):
            pass
        else:
            lock_file_fd = fd
            break
        time.sleep(0.05)
        current_time = time.time()
    if lock_file_fd is None:
        os.close(fd)
    return lock_file_fd


def release(lock_file_fd: int) -> None:
    """Release the acquired file lock.

    Args:
        lock_file_fd (int): File descriptor of the locked file.
    """
    fcntl.flock(lock_file_fd, fcntl.LOCK_UN)
    os.close(lock_file_fd)


def create_handle():
    """Initialize ROS node and connect to the Pro450 robot."""
    global mc
    rospy.init_node("mycobot_services")
    rospy.loginfo("Starting ultraArm service node...")
    port = rospy.get_param("~port", '/dev/ttyUSB0')
    baud = rospy.get_param("~baud", 1000000)
    rospy.loginfo("%s,%s" % (port, baud))
    mc = UltraArmP1(port, baud)
    mc.set_joint_enable(0)
    time.sleep(0.05)  # wait for serial port initialization
    # threading.Thread(target=read_angles_loop, daemon=True).start()
    # threading.Thread(target=read_coords_loop, daemon=True).start()


def create_services():
    """Create ROS services for robot control and start the service loop."""
    rospy.Service("set_joint_angles", SetAngles, set_angles)
    rospy.Service("get_joint_angles", GetAngles, get_angles)
    rospy.Service("set_joint_coords", SetCoords, set_coords)
    rospy.Service("get_joint_coords", GetCoords, get_coords)
    # rospy.Service("switch_gripper_status", GripperStatus, switch_status)
    rospy.loginfo("Services are ready")
    rospy.spin()

def read_angles_loop():
    global latest_angles, mc
    rate = rospy.Rate(30)   # 30 Hz
    while not rospy.is_shutdown():
        try:
            angles = mc.get_angles_info()
            # rospy.loginfo(f'get angle data: {angles}')
            if isinstance(angles, (list, tuple)) and len(angles) == 4:
                latest_angles = angles
        except:
            pass
        rate.sleep()
        
def read_coords_loop():
    global latest_coords, mc
    rate = rospy.Rate(30)   # 30 Hz
    while not rospy.is_shutdown():
        try:
            coords = mc.get_coords_info()
            # rospy.loginfo(f'get coords data: {coords}')
            if isinstance(coords, (list, tuple)) and len(coords) == 4:
                latest_coords = coords
        except:
            pass
        rate.sleep()

def set_angles(req: SetAngles) -> SetAnglesResponse:
    """Set the robot joint angles.

    Args:
        req (SetAngles): ROS service request with target angles and speed.

    Returns:
        SetAnglesResponse: Service response indicating success.
    """
    angles = [
        req.joint_1,
        req.joint_2,
        req.joint_3,
        req.joint_4,
    ]
    sp = req.speed
    angles = [round(i, 2) for i in angles]
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        rospy.loginfo(f'send angle data: {angles}')
        mc.set_angles(angles, sp, _async=False)
        release(lock)
        rospy.loginfo(f'done send angle data: {angles}')

    return SetAnglesResponse(True)

def get_angles_backup(req):
    global latest_angles
    if not mc:
        return GetAnglesResponse(0,0,0,0)
    
    return GetAnglesResponse(*latest_angles)


def get_angles(req: GetAngles) -> GetAnglesResponse:
    """Get the current robot joint angles.

    Args:
        req (GetAngles): Empty ROS service request.

    Returns:
        GetAnglesResponse: Service response with current angles.
    """
    global latest_angles
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        for i in range(3):
            angles = mc.get_angles_info()
            if angles != -1:
                break
        release(lock)
        time.sleep(0.05)
        if not isinstance(angles, (list, tuple)) or len(angles) != 4:
            # rospy.logwarn_throttle(5.0, f'Invalid angle data: {angles}; return latest valid angles: {latest_angles}')
            # 返回安全默认值，避免异常
            return GetAnglesResponse(*latest_angles)
        latest_angles = list(angles)
        return GetAnglesResponse(*angles)


def set_coords(req: SetCoords) -> SetCoordsResponse:
    """Set the robot end-effector coordinates.

    Args:
        req (SetCoords): ROS service request with target coordinates and speed.

    Returns:
        SetCoordsResponse: Service response indicating success.
    """
    coords = [
        req.x,
        req.y,
        req.z,
    ]
    sp = req.speed
    coords = [round(i, 2) for i in coords]
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        mc.set_coords(coords, sp, _async=False)
        release(lock)

    return SetCoordsResponse(True)

def get_coords_backup(req):
    if not mc:
        return GetCoordsResponse(0,0,0,0)
    global latest_coords
    return GetCoordsResponse(*latest_coords)

def get_coords(req: GetCoords) -> GetCoordsResponse:
    """Get the robot end-effector coordinates.

    Args:
        req (GetCoords): Empty ROS service request.

    Returns:
        GetCoordsResponse: Service response with current coordinates.
    """
    global latest_coords
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        for i in range(3):
            coords = mc.get_coords_info()
            if coords != -1:
                break
        release(lock)
        time.sleep(0.05)
        if not isinstance(coords, (list, tuple)) or len(coords) != 4:
            # rospy.logwarn_throttle(5.0, f'Invalid coord data: {coords}; return latest valid coords: {latest_coords}')
            # 返回安全默认值，避免异常
            return GetCoordsResponse(*latest_coords)
        latest_coords = list(coords)
        return GetCoordsResponse(*coords)


def build_robot_message():
    """Build robot limit information from pymycobot RobotLimit."""
    angles_min = ROBOT_LIMIT.get("angles_min", [-165, -18, 89, -179])
    angles_max = ROBOT_LIMIT.get("angles_max", [165, 85, 200, 179])
    lines = [
        "",
        "ultraArm P1 Status",
        "--------------------------------",
        "Joint Limit:",
    ]
    for index, (min_angle, max_angle) in enumerate(zip(angles_min, angles_max), start=1):
        lines.append(f"    joint {index}: {format_limit_value(min_angle)} ~ {format_limit_value(max_angle)}")
    return "\n".join(lines)


def output_robot_message():
    """Print robot status message to the console."""
    print(build_robot_message())


if __name__ == "__main__":
    create_handle()
    output_robot_message()
    create_services()
