#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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


def callback(data):
    
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(round(value, 3))
    data_list = data_list[:7]
    print("radians:%s" % data_list[:6])
    # t1 = time.time()
    mc.send_radians(data_list[:6], 25)
    # time.sleep(0.02)
    gripper_value = int(abs(-0.7 - data_list[6]) * 117)
    print("gripper_value:%s\n" % gripper_value)
    mc.set_gripper_value(gripper_value, 80)



def listener():
    global mc
    rospy.init_node("mycobot_reciver", anonymous=True)
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot280(port, baud)
    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)
    
    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止 python 退出，直到该节点停止
    rospy.spin()


if __name__ == "__main__":
    listener()
