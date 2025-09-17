#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
follow_display.py
This ROS node connects to a Pro450 robotic arm and publishes its joint states
and a visualization marker to ROS topics.

It periodically retrieves the robot's joint angles and coordinates,
converts angles to radians, and publishes the data to 'joint_states' topic.
It also publishes a spherical marker representing the end-effector position.

Author: WangWeiJian
Date: 2025-09-08
"""

import math
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

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


def talker():
    """Initialize ROS node and publish joint states and visualization marker.

    The function:
      - Connects to the Pro450 robotic arm.
      - Publishes joint angles to 'joint_states' topic.
      - Publishes a visualization marker to 'visualization_marker' topic.
      - Continuously updates at 30Hz until ROS is shutdown.
    """
    rospy.init_node("display", anonymous=True)

    print("Trying to connect to real MyCobot Pro450...")
    ip = rospy.get_param("~ip", "192.168.0.232")
    port = rospy.get_param("~port", 4500)
    print("IP: {}, port: {}\n".format(ip, port))

    try:
        mycobot_450 = Pro450Client(ip, port)
    except Exception as e:
        print(e)
        print(
            """\
            \rFailed to connect to MyCobot Pro450!
            \rPlease check if MyCobot Pro450 is connected.
            \rPlease check if the IP or port is correct.
        """
        )
        exit(1)

    # Enable all motors
    mycobot_450.set_motor_enabled(254, 0)
    time.sleep(0.1)
    print("All servos released.\n")

    # ROS publishers
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    pub_marker = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(30)  # 30Hz update rate

    # Initialize joint state message
    joint_state_send = JointState()
    joint_state_send.header = Header()
    joint_state_send.name = [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    ]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    # Initialize marker
    marker_ = Marker()
    marker_.header.frame_id = "/base"
    marker_.ns = "my_namespace"

    print("Publishing ...")
    while not rospy.is_shutdown():
        try:
            # Update joint state header timestamp
            joint_state_send.header.stamp = rospy.Time.now()

            # Get robot joint angles
            angles = mycobot_450.get_angles()
            if isinstance(angles, list) and len(angles) > 0:
                # Convert angles to radians for ROS
                data_list = [math.radians(value) for value in angles]
                joint_state_send.position = data_list
                pub.publish(joint_state_send)
            else:
                rospy.logwarn("Failed to get valid angles: {}".format(angles))

            # Get robot coordinates
            coords = mycobot_450.get_coords()
            if not isinstance(coords, list) or len(coords) == 0 or coords == -1:
                rospy.logwarn("Failed to get valid coordinates: {}".format(coords))
                coords = [0, 0, 0, 0, 0, 0]  # fallback

            # Update marker
            marker_.header.stamp = rospy.Time.now()
            marker_.type = marker_.SPHERE
            marker_.action = marker_.ADD
            marker_.scale.x = 0.04
            marker_.scale.y = 0.04
            marker_.scale.z = 0.04

            # Convert robot coordinates to meters and set marker position
            marker_.pose.position.x = coords[1] / 1000 * -1
            marker_.pose.position.y = coords[0] / 1000
            marker_.pose.position.z = coords[2] / 1000

            marker_.color.a = 1.0
            marker_.color.g = 1.0
            pub_marker.publish(marker_)

            rate.sleep()
        except Exception as e:
            print(e)


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
