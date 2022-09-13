#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import time
import rospy
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot


mc = None
gripper_value = []

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(round(value,3))
    data_list = data_list[:7]
    print("radians:%s"%data_list[:6])
    mc.send_radians(data_list[:6], 80)
    gripper_value = int(abs(-0.7 - data_list[6])* 117)
    print("gripper_value:%s"%gripper_value)
    mc.set_gripper_value(gripper_value, 80)


def listener():
    global mc
    global gripper_value 
    rospy.init_node("mycobot_reciver", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止 python 退出，直到该节点停止
    rospy.spin()


if __name__ == "__main__":
    listener()
