#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
sync_plan.py
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


def callback(data: JointState):
    """Callback function for ROS JointState subscription.

    This function converts incoming joint positions (radians) to angles
    in degrees and sends them to the Pro450 robotic arm.

    Args:
        data (JointState): Joint state message containing joint positions.
    """
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)

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
    global mc

    rospy.init_node("control_slider", anonymous=True)

    ip = rospy.get_param("~ip", "192.168.0.232")
    port = rospy.get_param("~port", 4500)
    print(ip, port)

    mc = Pro450Client(ip, port)
    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)

    rospy.Subscriber("joint_states", JointState, callback)

    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
