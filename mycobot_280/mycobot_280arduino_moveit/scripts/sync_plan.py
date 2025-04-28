#!/usr/bin/env python2
import math
import time
import rospy
from sensor_msgs.msg import JointState

import pymycobot
from packaging import version
# min low version require
MAX_REQUIRE_VERSION = '3.6.1'
current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280


mc = None


def callback(data):
    global latest_data
    latest_data = [round(math.degrees(value), 2) for value in data.position]
    rospy.loginfo(f"Joint angles: {latest_data}")

def control_loop(event):
    if latest_data:
        rospy.loginfo(f"Sending angles: {latest_data}")
        mc.send_angles(latest_data, 25)

def listener():
    global mc
    rospy.init_node("control_slider", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot280(port, baud)
    time.sleep(1) # open port,need wait
    
    rospy.Subscriber("joint_states", JointState, callback)

    # 启动定时器，每0.5秒执行一次控制循环
    rospy.Timer(rospy.Duration(0.5), control_loop)

    rospy.spin()


if __name__ == "__main__":
    listener()
