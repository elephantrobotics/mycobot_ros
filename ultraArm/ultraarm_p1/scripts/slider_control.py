#!/usr/bin/env python3

"""[summary]
control_slider.py
This ROS node subscribes to JointState messages and sends the corresponding
angles to a Pro450 robotic arm using pymycobot.

It converts radians from JointState into degrees and publishes them
to the robot through the UltraArmP1.

Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyUSB0'
    baud: serial prot baudrate. Defaults is 115200.
    

Author: WangWeiJian
Date: 2025-11-24
"""

import time
import rospy
from sensor_msgs.msg import JointState

import math
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '4.0.3'

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
    from pymycobot import UltraArmP1

ua = None


def callback(data):
    """Callback function for ROS JointState subscription.

    This function converts incoming joint positions (radians) to angles
    in degrees and sends them to the ultarArm P1 robotic arm.

    Args:
        data (JointState): Joint state message containing joint positions.
    """
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)
        
    joint1 = data_list[0]
    joint2 = data_list[1]
    joint3 = data_list[5] + 90
    joint4 = data_list[-1]
    angles_list = [joint1, joint2, joint3, joint4]
        
    rospy.loginfo(rospy.get_caller_id() + "%s", angles_list)
    ua.set_angles(angles_list, 2800, _async=False)


def listener():
    global ua
    rospy.init_node("control_slider", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyUSB0") # Select connected device. 选择连接设备
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    ua = UltraArmP1(port, baud)
    
    rospy.Subscriber("joint_states", JointState, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
