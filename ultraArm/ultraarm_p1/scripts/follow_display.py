#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
follow_display.py
This ROS node connects to a ultraArmP1 robotic arm and publishes its joint states
to ROS topics.

It periodically retrieves the robot's joint angles, converts angles to radians,
and publishes the data to 'joint_states' topic.

Author: WangWeiJian
Date: 2025-11-24
"""

import math
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

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


def valid_angles(angles):
    """Accept any finite 4-joint reading for model follow (no software joint limits)."""
    if not isinstance(angles, list) or len(angles) != 4:
        return False
    try:
        return all(math.isfinite(float(angle)) for angle in angles)
    except (TypeError, ValueError):
        return False


def angles_to_joint_positions(angles):
    display_angles = list(angles)
    display_angles[2] -= 90
    return [math.radians(value) for value in display_angles]


def talker():
    """Initialize ROS node and publish joint states.

    The function:
      - Connects to the UltraArm P1 robotic arm.
      - Publishes joint angles to 'joint_states' topic.
      - Continuously updates until ROS is shutdown.
    """
    rospy.init_node("display", anonymous=True)

    rospy.loginfo("Trying to connect to real ultraArm P1...")
    port = rospy.get_param("~port", "/dev/ttyUSB0")  # Select connected device. 选择连接设备
    baud = rospy.get_param("~baud", 1000000)
    publish_rate = max(float(rospy.get_param("~publish_rate", 30.0)), 0.1)
    rospy.loginfo("port: {}, baud: {}\n".format(port, baud))

    try:
        ua = UltraArmP1(port, baud)
        time.sleep(0.05)
    except Exception as e:
        rospy.loginfo(e)
        rospy.loginfo(
            """\
            \rFailed to connect to ultraArm P1!
            \rPlease check if ultraArm P1 is connected.
            \rPlease check if the IP or port is correct.
        """
        )
        exit(1)

    ua.set_end_button_enable()
    rospy.loginfo(
        'Please press the LED button at the end of the machine to drag the joint.\n'
        '请按下机器末端LED按钮进行关节拖拽运动\n'
    )

    # ROS publishers
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    rate = rospy.Rate(publish_rate)

    # Initialize joint state message
    joint_state_send = JointState()
    joint_state_send.header = Header()
    joint_state_send.name = [
        "J1", "J2", "J3", "J4"
    ]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []
    last_valid_positions = None

    rospy.loginfo("Publishing ...")
    while not rospy.is_shutdown():
        try:
            # Update joint state header timestamp
            joint_state_send.header.stamp = rospy.Time.now()

            # Get robot joint angles
            angles = ua.get_angles_info()
            if valid_angles(angles):
                last_valid_positions = angles_to_joint_positions(angles)
            else:
                if last_valid_positions is None:
                    rate.sleep()
                    continue

            joint_state_send.position = list(last_valid_positions)
            pub.publish(joint_state_send)
            rate.sleep()
        except Exception as e:
            import traceback
            e = traceback.format_exc()
            print(e)


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
