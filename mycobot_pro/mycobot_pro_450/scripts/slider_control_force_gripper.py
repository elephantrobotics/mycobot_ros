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
gripper_value = None

def callback(data: JointState):
    """Callback function for ROS JointState subscription.

    This function converts incoming joint positions (radians) to angles
    in degrees and sends them to the Pro450 robotic arm and force gripper.

    Args:
        data (JointState): Joint state message containing joint positions.
    """
    global mc, gripper_value, angles_queue
    data_list = []
    gripper_value = 0

    for i, value in enumerate(data.position):
        if i < 6:
            # Convert radians to degrees for the robot joints
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
        else:
            # Map force gripper value to 0-100 scale
            mapped_value = value * 100
            gripper_value = int(round(mapped_value, 2))

    rospy.loginfo(f'joint_list: {data_list}, gripper_value: {gripper_value}')
    
    # mc.send_angles(data_list, 25)
    # mc.set_pro_gripper_angle(gripper_value)
    angles_queue = data_list
    gripper_value = gripper_value

def _send_loop():
    """Background loop to send angles and gripper commands to the robotic arm.
    """
    global mc, gripper_value, angles_queue
    while True:
        time.sleep(0.01)
        if angles_queue:
            mc.send_angles(angles_queue, 25)
            angles_queue = None
        if gripper_value is not None:
            mc.set_pro_gripper_angle(gripper_value)
            gripper_value = None
def listener():
    """Initialize ROS node and Pro450Client, then subscribe to JointState topic.

    The function:
      - Initializes the ROS node.
      - Connects to the Pro450 robotic arm using IP and port.
      - Sets fresh mode for the robotic arm.
      - Subscribes to the "joint_states" topic and listens for updates.
    """
    global mc, gripper_value, angles_queue

    rospy.init_node("control_slider", anonymous=True)

    ip = rospy.get_param("~ip", "192.168.0.232")
    port = rospy.get_param("~port", 4500)
    print(ip, port)

    mc = Pro450Client(ip, port)
    time.sleep(0.05)
    if mc.is_power_on !=1:
        mc.power_on()
    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)
    mc.set_limit_switch(2, 0)


    threading.Thread(target=_send_loop, daemon=True).start()

    rospy.Subscriber("joint_states", JointState, callback)

    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
