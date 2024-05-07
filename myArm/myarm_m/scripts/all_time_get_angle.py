#!/usr/bin/env python3
# encoding:utf-8
from pymycobot import MyArmM
from sensor_msgs.msg import JointState
import rospy
from math import pi
import time
# 1：[-174, 167]
# 2:[-92, 93]
# 3:[-91,103]
# 4:[-170, 170]
# 5:[-96,89]
# 6:[-170,170]

global mam
mam = MyArmM('/dev/ttyACM0', debug=False)
while True:
    print('当前角度：', mam.get_joints_angle())
