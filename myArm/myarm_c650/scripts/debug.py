
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
import rosnode
import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose

global mam
mam = MyArmC('/dev/ttyACM0', debug=False)
angle = mam.get_joints_angle()
print(angle)
mam.set_tool_led_color(255,255,255)
# for i in range(8):
#     mam.set_servo_calibrate(i)
#     time.sleep(0.5)


