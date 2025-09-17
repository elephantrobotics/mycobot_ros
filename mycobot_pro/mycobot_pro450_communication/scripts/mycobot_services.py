#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mycobot_services.py
This ROS node provides service interfaces for controlling a Pro450 robotic arm.
It includes services to:
    - Set/Get joint angles
    - Set/Get end-effector coordinates
    - Switch gripper status

The node ensures serial port safety using file locking to prevent conflicts
when multiple processes access the robot simultaneously.

Author: WangWeiJian
Date: 2025-09-08
"""

import time
import rospy
import os
import fcntl
from mycobot_pro450_communication.srv import *

import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '3.9.9'

current_verison = pymycobot.__version__
print('Current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'The current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import Pro450Client

mc = None


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
        time.sleep(1.0)
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
    rospy.loginfo("Starting MyCobot service node...")
    ip = rospy.get_param("~ip", '192.168.0.232')
    port = rospy.get_param("~port", 4500)
    rospy.loginfo("%s,%s" % (ip, port))
    mc = Pro450Client(ip, port)
    time.sleep(0.05)  # wait for serial port initialization


def create_services():
    """Create ROS services for robot control and start the service loop."""
    rospy.Service("set_joint_angles", SetAngles, set_angles)
    rospy.Service("get_joint_angles", GetAngles, get_angles)
    rospy.Service("set_joint_coords", SetCoords, set_coords)
    rospy.Service("get_joint_coords", GetCoords, get_coords)
    rospy.Service("switch_gripper_status", GripperStatus, switch_status)
    rospy.loginfo("Services are ready")
    rospy.spin()


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
        req.joint_5,
        req.joint_6,
    ]
    sp = req.speed

    if mc:
        lock = acquire("/tmp/mycobot_lock")
        mc.send_angles(angles, sp)
        release(lock)

    return SetAnglesResponse(True)


def get_angles(req: GetAngles) -> GetAnglesResponse:
    """Get the current robot joint angles.

    Args:
        req (GetAngles): Empty ROS service request.

    Returns:
        GetAnglesResponse: Service response with current angles.
    """
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        angles = mc.get_angles()
        release(lock)
        if angles is None:
            rospy.logwarn('No angle data available')
            return GetAnglesResponse()
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
        req.rx,
        req.ry,
        req.rz,
    ]
    sp = req.speed

    if mc:
        lock = acquire("/tmp/mycobot_lock")
        mc.send_coords(coords, sp)
        release(lock)

    return SetCoordsResponse(True)


def get_coords(req: GetCoords) -> GetCoordsResponse:
    """Get the robot end-effector coordinates.

    Args:
        req (GetCoords): Empty ROS service request.

    Returns:
        GetCoordsResponse: Service response with current coordinates.
    """
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        coords = mc.get_coords()
        release(lock)
        if coords is None:
            rospy.logwarn('No coordinate data available')
            return GetCoordsResponse()
        return GetCoordsResponse(*coords)


def switch_status(req: GripperStatus) -> GripperStatusResponse:
    """Switch the gripper open/close status.

    Args:
        req (GripperStatus): ROS service request with desired gripper status.

    Returns:
        GripperStatusResponse: Service response indicating success.
    """
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        if req.Status:
            mc.set_pro_gripper_open()
        else:
            mc.set_pro_gripper_close()
        release(lock)

    return GripperStatusResponse(True)


robot_msg = """
MyCobot Status
--------------------------------
Joint Limit:
    joint 1: -165 ~ +165
    joint 2: -120 ~ +120
    joint 3: -158 ~ +158
    joint 4: -165 ~ +165
    joint 5: -165 ~ +165
    joint 6: -175 ~ +175
"""


def output_robot_message():
    """Print robot status message to the console."""
    connect_status = False
    servo_infomation = "unknown"
    servo_temperature = "unknown"
    atom_version = "unknown"

    print(robot_msg)


if __name__ == "__main__":
    create_handle()
    output_robot_message()
    create_services()
