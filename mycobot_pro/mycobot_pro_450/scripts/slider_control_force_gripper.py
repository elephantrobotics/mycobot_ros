#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
control_slider_force_gripper.py
This ROS node subscribes to JointState messages and sends the corresponding
angles to a Pro450 robotic arm and force gripper using pymycobot.

It converts radians from JointState into degrees and publishes them
to the robot through the Pro450Client.

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


mc = None
angles_queue = None
pending_gripper_value = None
command_lock = threading.Lock()
joint5_offset_deg = 0.0
joint5_firmware_limits = None
last_sent_angles = None
last_sent_gripper_value = None
last_rejected_joint5_angle = None


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
    """Callback function for ROS JointState subscription.

    This function converts incoming joint positions (radians) to angles
    in degrees and sends them to the Pro450 robotic arm and force gripper.

    Args:
        data (JointState): Joint state message containing joint positions.
    """
    global angles_queue, pending_gripper_value
    global last_sent_angles, last_sent_gripper_value, last_rejected_joint5_angle
    if joint5_offset_deg is None:
        rospy.logerr_once("Refusing J5 command because the selected URDF profile is incompatible with firmware limits")
        return
    data_list = []
    gripper_command = None

    for i, value in enumerate(data.position):
        if i < 6:
            # Convert radians to degrees for the robot joints
            radians_to_angles = round(math.degrees(value), 2)
            if i == 4:
                radians_to_angles -= joint5_offset_deg
            data_list.append(radians_to_angles)
    # Mimic joints may have negative positions.  Only the active controller
    # joint represents a gripper command; never infer it from an array index.
    if "gripper_controller" in data.name:
        gripper_index = data.name.index("gripper_controller")
        if gripper_index < len(data.position):
            gripper_position = data.position[gripper_index]
            gripper_command = int(round(max(0.0, min(1.0, gripper_position)) * 100))

    if joint5_firmware_limits is None:
        rospy.logerr_once("Refusing J5 command because firmware limits are unavailable")
        return
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

    last_sent_angles = list(data_list)
    
    if gripper_command is not None:
        last_sent_gripper_value = gripper_command
    rospy.loginfo(
        "joint_list: %s, gripper_value: %s",
        data_list, gripper_command if gripper_command is not None else "unchanged",
    )
    with command_lock:
        angles_queue = data_list
        if gripper_command is not None:
            pending_gripper_value = gripper_command

def _send_loop():
    """Background loop to send angles and gripper commands to the robotic arm.
    """
    global mc, pending_gripper_value, angles_queue
    global joint5_offset_deg, joint5_firmware_limits
    while True:
        time.sleep(0.01)
        with command_lock:
            angles_to_send = angles_queue
            gripper_to_send = pending_gripper_value
            angles_queue = None
            pending_gripper_value = None
        if angles_to_send is not None:
            mc.send_angles(angles_to_send, 25)
        if gripper_to_send is not None:
            mc.set_pro_gripper_angle(gripper_to_send)
def listener():
    """Initialize ROS node and Pro450Client, then subscribe to JointState topic.

    The function:
      - Initializes the ROS node.
      - Connects to the Pro450 robotic arm using IP and port.
      - Sets fresh mode for the robotic arm.
      - Subscribes to the "joint_states" topic and listens for updates.
    """
    global mc, pending_gripper_value, angles_queue
    global joint5_offset_deg, joint5_firmware_limits
    rospy.init_node("control_slider", anonymous=True)

    ip = rospy.get_param("~ip", "192.168.0.232")
    port = rospy.get_param("~port", 4500)
    joint5_offset_deg = rospy.get_param("/mycobot_pro_450/joint5_offset_deg", 0.0)
    print(ip, port)

    mc = Pro450Client(ip, port)
    time.sleep(0.05)
    if mc.is_power_on !=1:
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
