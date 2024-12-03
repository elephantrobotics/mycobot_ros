#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyUSB0'
    baud: serial prot baudrate. Defaults is 115200.
"""

import rospy
import time
from sensor_msgs.msg import JointState
import math
import pymycobot
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '3.6.4'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot320 import MyCobot320


mc = None
gripper_value = []

def callback(data):
    global mc
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(round(value, 3))

    data_list = data_list[:7]
    angles_list = [round(math.degrees(radian), 2) for radian in data_list[:6]]
    print("angles:%s"%angles_list)
    mc.send_angles(angles_list, 25)
    gripper_value = int(data_list[6]* 100)
    print("gripper_value:%s"%gripper_value)

    mc.set_pro_gripper_angle(14, gripper_value)
    
def listener():
    global mc
    global gripper_value
   
    rospy.init_node("control_slider", anonymous=True)

    
    port = rospy.get_param("~port", "/dev/ttyACM0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot320(port, baud)
    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)
    
    rospy.Subscriber("joint_states", JointState, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()