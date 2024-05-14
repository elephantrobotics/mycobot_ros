
#!/usr/bin/env python

"""
This package need `pymycobot`.
This file for test the API if right.

Just can run in Linux.
"""


from math import pi
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot.myarmc import MyArmC
from pymycobot.myarmm import MyArmM
import rosnode
import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose

global mam
mam = MyArmM('/dev/ttyACM1', debug=False)
angle = mam.get_joints_angle()
print(angle)
# mam.set_tool_led_color(255,255,255)
# for i in range(8):
#     mam.set_servo_calibrate(i)
#     time.sleep(0.5)

# def linear_transform(x):
#     # 两个已知数据点
#     x1, y1 = -89.5, 0.022
#     x2, y2 = 0, 0
    
#     # 计算斜率
#     m = (y2 - y1) / (x2 - x1)
    
#     # 计算截距
#     c = y1 - m * x1
    
#     # 应用线性变换
#     y = m * x + c
    
#     return y

# # 测试
# print(linear_transform(-89.5))  # 应该输出0.022
# print(linear_transform(0))      # 应该输出0




