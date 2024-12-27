#!/usr/bin/env python
# encoding=utf-8
import time
import math
import rospy
from sensor_msgs.msg import JointState

import pymycobot
from packaging import version
# min low version require
MAX_REQUIRE_VERSION = '3.5.3'
current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot import MyCobot


mc = None


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)
        
    rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
    mc.send_angles(data_list, 25)


def listener():
    global mc
    rospy.init_node("mycobot_reciver", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyAMA0")
    baud = rospy.get_param("~baud", 1000000)
    print(port, baud)
    mc = MyCobot(port, baud)
    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)
    
    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止 python 退出，直到该节点停止
    rospy.spin()


if __name__ == "__main__":
    listener()
