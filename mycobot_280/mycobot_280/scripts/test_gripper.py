#!/usr/bin/env python2
# encoding:utf-8
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from pymycobot.mycobot import MyCobot

mc = MyCobot("/dev/ttyUSB0", 115200)
# print(mc.is_gripper_moving())
# time.sleep(2)
# print(mc.get_gripper_value())
# print(mc.get_radians())
# mc.set_gripper_state(0,50)
# time.sleep(2)
mc.set_gripper_value(0,50)
time.sleep(2)


