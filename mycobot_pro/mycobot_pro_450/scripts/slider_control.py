#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
control_slider.py
This ROS node subscribes to JointState messages and sends the corresponding
angles to a Pro450 robotic arm using pymycobot.

It converts radians from JointState into degrees and publishes them
to the robot through the Pro450Client.

Author: WangWeiJian
Date: 2025-09-08
"""

import math
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
joint5_offset_deg = 0.0
joint5_firmware_limits = None
last_sent_angles = None
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
    in degrees and sends them to the Pro450 robotic arm.

    Args:
        data (JointState): Joint state message containing joint positions.
    """
    global last_sent_angles, last_rejected_joint5_angle
    if joint5_offset_deg is None:
        rospy.logerr_once("Refusing J5 command because the selected URDF profile is incompatible with firmware limits")
        return
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        if index == 4:
            radians_to_angles -= joint5_offset_deg
        data_list.append(radians_to_angles)

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
    if last_sent_angles is not None and all(
        abs(current - previous) < 0.01
        for current, previous in zip(data_list, last_sent_angles)
    ):
        return

    last_sent_angles = list(data_list)
    rospy.loginfo(data_list)
    mc.send_angles(data_list, 25)


def listener():
    """Initialize ROS node and Pro450Client, then subscribe to JointState topic.

    The function:
      - Initializes the ROS node.
      - Connects to the Pro450 robotic arm using IP and port.
      - Sets fresh mode for the robotic arm.
      - Subscribes to the "joint_states" topic and listens for updates.
    """
    global mc, joint5_offset_deg, joint5_firmware_limits

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

    rospy.Subscriber("joint_states", JointState, callback)

    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
