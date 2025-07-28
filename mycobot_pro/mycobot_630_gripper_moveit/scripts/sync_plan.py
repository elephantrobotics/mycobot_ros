#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from socket import *
import math
import sys
import time
from multiprocessing import Lock

import rospy
from sensor_msgs.msg import JointState

from pymycobot.elephantrobot import ElephantRobot

global mc


def callback(data):
    """callback function,回调函数"""
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)

    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)
        
    rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
    # mc.write_angles(data_list, 800)
    
    for joint,angle in enumerate(data_list[:6]):
        mc.write_angle(joint,angle,800)
        
    for gripper_angle in data_list[6:]:
        mc.set_cag_gripper_value(gripper_angle,800)
    

    
    # excepted_joint_count = 13
    # if len(data_list) == excepted_joint_count:
    #     mc.write_angles(data_list, 800)
    # else:
    #     rospy.logwarn("关节数量不匹配，期望数量是%d,实际数量是%d",excepted_joint_count,len(data_list))


def listener():
    global mc
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
