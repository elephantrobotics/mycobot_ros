#!/usr/bin/env python3
# encoding:utf-8
"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control_parallel_gripper.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyTHS1'
    baud: serial prot baudrate. Defaults is 1000000.
"""
import time
import math
import rospy
from sensor_msgs.msg import JointState

import pymycobot
from packaging import version
# min low version require
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280


mc = None


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(round(value,3))
    # print(data_list[6:])
    
    data_list = data_list[:7]
    print("radians:%s"%data_list[:6])
    mc.send_radians(data_list[:6], 25)
    # 线性插值公式 gripper_value = (current_value - min_value) / (max_value - min_value) * 100
    gripper_value = int((data_list[6] - (-0.007)) / (0 - (-0.007)) * 100)
    print("gripper_value:%s"%gripper_value)
    mc.set_gripper_value(gripper_value, 80, 3)


def listener():
    global mc
    rospy.init_node("control_slider", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyTHS1")
    baud = rospy.get_param("~baud", 1000000)
    print(port, baud)
    mc = MyCobot280(port, baud)
    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)

    rospy.Subscriber("joint_states", JointState, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    # spin()只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
