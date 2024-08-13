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
from socket import *
import math
import sys
import time
from multiprocessing import Lock

import rospy
from sensor_msgs.msg import JointState

from pymycobot.elephantrobot import ElephantRobot
import rospy
import time
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot


mc = None
gripper_value = []

def callback(data):
    global mc
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(round(value, 3))

    data_list = data_list[:7]
    print("radians:%s"%data_list[:6])
    mc.send_radians(data_list[:6], 80)
    gripper_value = int(abs(-0.7-data_list[6])* 117)
    print("gripper_value:%s"%gripper_value)
    # mc.set_gripper_mode(0)
    # time.sleep(1)
    mc.set_gripper_value(gripper_value, 100)
    
def listener():
    global mc
    global gripper_value
    rospy.init_node("control_slider", anonymous=True)

    ip = rospy.get_param("~ip", "192.168.1.161")
    port = rospy.get_param("~port", 5001)
    print (ip, port)
    mc = ElephantRobot(ip, int(port))
    # START CLIENT,启动客户端
    res = mc.start_client()
    if res != "":
        sys.exit(1)

    mc.set_speed(90)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # spin()只是阻止python退出，直到该节点停止
    print ("sping ...")
    rospy.spin()


if __name__ == "__main__":
    listener()