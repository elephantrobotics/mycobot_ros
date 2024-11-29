#!/usr/bin/env python2
# -*- coding:utf-8 -*-
import time
import subprocess
import rospy
from sensor_msgs.msg import JointState

import pymycobot
from packaging import version

# min low version require
MAX_REQUIRE_VERSION = '3.6.0'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    raise RuntimeError('{}The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot import MyCobot


mc = None


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        # if index != 2:
        #     value *= -1
        data_list.append(value)

    mc.send_radians(data_list, 80)


def listener():
    global mc
    rospy.init_node("mycobot_reciver", anonymous=True)
    port = subprocess.check_output(['echo -n /dev/ttyUSB*'], 
                                    shell=True)

    port = rospy.get_param("~port", port)
    baud = rospy.get_param("~baud", 115200)
    # 1000000
    mc = MyCobot(port, baud)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()
