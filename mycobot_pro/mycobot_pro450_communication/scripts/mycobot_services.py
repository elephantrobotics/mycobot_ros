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
MIN_REQUIRE_VERSION = '4.0.1'

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
joint5_firmware_limits = None
joint5_offset_deg = 0.0

PROFILE_PARAM = "/mycobot_pro_450/joint5_offset_deg"


def read_joint5_firmware_limits(client):
    """Read J5's software limits from the connected controller."""
    try:
        lower = client.get_joint_min_angle(5)
        upper = client.get_joint_max_angle(5)
    except Exception as exc:
        rospy.logerr("Unable to read J5 firmware limits: %s", exc)
        return None
    if not isinstance(lower, (int, float)) or not isinstance(upper, (int, float)) or lower >= upper:
        rospy.logerr("Invalid J5 firmware limits returned by controller: %r, %r", lower, upper)
        return None
    rospy.loginfo("J5 firmware limits: [%.2f, %.2f] degrees", lower, upper)
    return lower, upper


def resolve_joint5_offset(limits, configured_offset):
    """Require the controller limits to match the selected URDF profile."""
    lower, upper = limits
    if abs(configured_offset) < 0.01:
        expected_lower, expected_upper = -162.0, 162.0
    elif abs(configured_offset - 90.0) < 0.01:
        expected_lower, expected_upper = -252.0, 72.0
    else:
        rospy.logerr("Unsupported J5 offset configured: %.2f degrees", configured_offset)
        return None
    if abs(lower - expected_lower) < 0.01 and abs(upper - expected_upper) < 0.01:
        return configured_offset
    rospy.logerr(
        "J5 firmware limits [%.2f, %.2f] do not match the selected URDF profile; expected [%.2f, %.2f]",
        lower, upper, expected_lower, expected_upper,
    )
    return None


def to_firmware_angles(angles):
    """Convert ROS logical joint angles to the firmware coordinate system."""
    firmware_angles = list(angles)
    if joint5_offset_deg is None:
        rospy.logerr("Refusing J5 command because the selected URDF profile is incompatible with firmware limits")
        return None
    firmware_angles[4] -= joint5_offset_deg
    if joint5_firmware_limits is None:
        rospy.logerr("Refusing J5 command because firmware limits are unavailable")
        return None
    lower, upper = joint5_firmware_limits
    if not lower <= firmware_angles[4] <= upper:
        rospy.logerr(
            "Refusing J5 command %.2f degrees outside firmware limits [%.2f, %.2f]",
            firmware_angles[4], lower, upper,
        )
        return None
    return firmware_angles


def from_firmware_angles(angles):
    """Convert firmware joint angles to ROS logical joint coordinates."""
    logical_angles = list(angles)
    if joint5_offset_deg is None:
        return logical_angles
    logical_angles[4] += joint5_offset_deg
    return logical_angles

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
    global mc, joint5_firmware_limits, joint5_offset_deg
    rospy.init_node("mycobot_services")
    rospy.loginfo("Starting MyCobot service node...")
    ip = rospy.get_param("~ip", '192.168.0.232')
    port = rospy.get_param("~port", 4500)
    rospy.loginfo("%s,%s" % (ip, port))
    mc = Pro450Client(ip, port)
    time.sleep(0.05)  # wait for serial port initialization
    if mc.is_power_on !=1:
        mc.power_on()
    time.sleep(0.05)
    if mc.get_fresh_mode() !=0:
        mc.set_fresh_mode(0)
    time.sleep(0.05)
    mc.set_limit_switch(2, 0)
    time.sleep(0.05)
    joint5_firmware_limits = read_joint5_firmware_limits(mc)
    if joint5_firmware_limits is not None:
        joint5_offset_deg = resolve_joint5_offset(
            joint5_firmware_limits, rospy.get_param(PROFILE_PARAM, 0.0)
        )


def create_services():
    """Create ROS services for robot control and start the service loop."""
    rospy.Service("set_joint_angles", SetAngles, set_angles)
    rospy.Service("get_joint_angles", GetAngles, get_angles)
    rospy.Service("set_joint_coords", SetCoords, set_coords)
    rospy.Service("get_joint_coords", GetCoords, get_coords)
    rospy.Service("switch_gripper_status", GripperStatus, switch_status)
    rospy.Service("get_gripper_angle", GetGripperValue, get_gripper_angle)
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
    firmware_angles = to_firmware_angles(angles)
    if firmware_angles is None:
        return SetAnglesResponse(False)
    sp = req.speed

    if mc:
        lock = acquire("/tmp/mycobot_lock")
        mc.send_angles(firmware_angles, sp)
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
        return GetAnglesResponse(*from_firmware_angles(angles))
    
def get_gripper_angle(req: GetGripperValue) -> GetGripperValueResponse:
    """Get the current robot joint angles.

    Args:
        req (GetAngles): Empty ROS service request.

    Returns:
        GetAnglesResponse: Service response with current angles.
    """
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        gripper_angle = mc.get_pro_gripper_angle()
        release(lock)
        if gripper_angle is None:
            rospy.logwarn('No gripper angle data available')
            return GetGripperValueResponse()
        return GetGripperValueResponse(gripper_angle)


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


def format_joint_limit(value):
    """Format a controller limit without unnecessary decimal places."""
    return "{:+g}".format(value)


def output_robot_message():
    """Print the six joint limits read from the connected controller."""
    lines = ["", "MyCobot Status", "--------------------------------", "Joint Limit:"]
    for joint_id in range(1, 7):
        try:
            lower = mc.get_joint_min_angle(joint_id)
            upper = mc.get_joint_max_angle(joint_id)
            if not isinstance(lower, (int, float)) or not isinstance(upper, (int, float)):
                raise ValueError("controller returned a non-numeric limit")
            lines.append(
                "    joint {}: {} ~ {}".format(
                    joint_id, format_joint_limit(lower), format_joint_limit(upper)
                )
            )
        except Exception as exc:
            lines.append("    joint {}: unavailable ({})".format(joint_id, exc))
    print("\n".join(lines))



if __name__ == "__main__":
    create_handle()
    output_robot_message()
    create_services()
