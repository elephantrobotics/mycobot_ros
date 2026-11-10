#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
sync_plan.py
This ROS node subscribes to JointState messages and sends the corresponding
angles to a Pro450 robotic arm using pymycobot. If joint_states contains
gripper_controller, F100 gripper commands are also sent automatically.

It converts radians from JointState into degrees and publishes them
to the robot through the Pro450Client. For V2, joint5 is converted from
ROS/URDF logical coordinates to firmware coordinates using the shared
/mycobot_pro_450/joint5_offset_deg parameter. Firmware J5 limits are checked
against the selected URDF profile before commands are sent.

Author: WangWeiJian
Date: 2025-09-08
"""

import math
import threading
import time
import rospy
from sensor_msgs.msg import JointState

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


ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT_NAME = "gripper_controller"

mc = None
angles_queue = None
pending_gripper_value = None
command_lock = threading.Lock()
joint5_offset_deg = 0.0
joint5_firmware_limits = None
last_sent_angles = None
last_sent_gripper_value = None
last_rejected_joint5_angle = None
gripper_mode_logged = False


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


def callback(data: JointState):
    """Callback for JointState: send arm angles and optional F100 gripper."""
    global angles_queue, pending_gripper_value
    global last_sent_angles, last_sent_gripper_value, last_rejected_joint5_angle
    global gripper_mode_logged

    if joint5_offset_deg is None:
        rospy.logerr_once(
            "Refusing J5 command because the selected URDF profile is incompatible with firmware limits"
        )
        return

    if joint5_firmware_limits is None:
        rospy.logerr_once("Refusing J5 command because firmware limits are unavailable")
        return

    if len(data.name) != len(data.position):
        rospy.logwarn_throttle(
            5.0,
            "Ignoring JointState with mismatched name/position lengths: %d names, %d positions",
            len(data.name),
            len(data.position),
        )
        return

    name_to_position = dict(zip(data.name, data.position))
    data_list = []
    for name in ARM_JOINT_NAMES:
        if name not in name_to_position:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring JointState missing required joint '%s'",
                name,
            )
            return
        angle_deg = round(math.degrees(name_to_position[name]), 2)
        if name == "joint5":
            angle_deg -= joint5_offset_deg
        data_list.append(angle_deg)

    # Auto-detect F100: only the active gripper_controller joint is commanded.
    gripper_command = None
    if GRIPPER_JOINT_NAME in name_to_position:
        if not gripper_mode_logged:
            rospy.loginfo(
                "Detected '%s' in joint_states; enabling F100 gripper sync",
                GRIPPER_JOINT_NAME,
            )
            gripper_mode_logged = True
        gripper_position = name_to_position[GRIPPER_JOINT_NAME]
        gripper_command = int(round(max(0.0, min(1.0, gripper_position)) * 100))

    lower, upper = joint5_firmware_limits
    if not lower <= data_list[4] <= upper:
        if last_rejected_joint5_angle != data_list[4]:
            rospy.logerr(
                "Refusing J5 command %.2f degrees outside firmware limits [%.2f, %.2f]",
                data_list[4], lower, upper,
            )
            last_rejected_joint5_angle = data_list[4]
        return

    last_rejected_joint5_angle = None
    angles_unchanged = last_sent_angles is not None and all(
        abs(current - previous) < 0.01
        for current, previous in zip(data_list, last_sent_angles)
    )
    gripper_unchanged = (
        gripper_command is None or gripper_command == last_sent_gripper_value
    )
    if angles_unchanged and gripper_unchanged:
        return

    if not angles_unchanged:
        last_sent_angles = list(data_list)

    if gripper_command is not None:
        last_sent_gripper_value = gripper_command

    if gripper_command is None:
        rospy.loginfo(data_list)
    else:
        rospy.loginfo("joint_list: %s, gripper_value: %s", data_list, gripper_command)

    with command_lock:
        if not angles_unchanged:
            angles_queue = data_list
        if gripper_command is not None:
            pending_gripper_value = gripper_command


def _send_loop():
    """Background loop to send arm and optional gripper commands."""
    global mc, pending_gripper_value, angles_queue
    while not rospy.is_shutdown():
        time.sleep(0.01)
        with command_lock:
            angles_to_send = angles_queue
            gripper_to_send = pending_gripper_value
            angles_queue = None
            pending_gripper_value = None
        # Prioritize gripper Modbus so arm streaming does not starve it.
        if gripper_to_send is not None:
            mc.set_pro_gripper_angle(gripper_to_send)
        if angles_to_send is not None:
            mc.send_angles(angles_to_send, 25)


def listener():
    """Initialize ROS node, connect to Pro450, and sync from joint_states."""
    global mc, joint5_offset_deg, joint5_firmware_limits

    rospy.init_node("sync_plan", anonymous=True)

    ip = rospy.get_param("~ip", "192.168.0.232")
    port = rospy.get_param("~port", 4500)
    joint5_offset_deg = rospy.get_param("/mycobot_pro_450/joint5_offset_deg", 0.0)
    print(ip, port)
    rospy.loginfo("joint5_offset_deg=%.2f", joint5_offset_deg)
    rospy.loginfo(
        "Gripper sync auto-detects '%s' in /joint_states",
        GRIPPER_JOINT_NAME,
    )

    mc = Pro450Client(ip, port)
    time.sleep(0.05)
    if mc.is_power_on != 1:
        mc.power_on()
    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)
    mc.set_limit_switch(2, 0)
    joint5_firmware_limits = read_joint5_firmware_limits(mc)
    if joint5_firmware_limits is not None:
        joint5_offset_deg = resolve_joint5_offset(
            joint5_firmware_limits, joint5_offset_deg
        )

    threading.Thread(target=_send_loop, daemon=True).start()

    rospy.Subscriber("joint_states", JointState, callback)

    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
