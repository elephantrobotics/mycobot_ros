#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *
import math
import sys
import time
from multiprocessing import Lock

import rospy
from sensor_msgs.msg import JointState

from pymycobot import Pro630Client
# from pro630client import Pro630Client

global mc


def callback(data):
    """callback function,回调函数"""
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)
        
    # rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
    mc.send_angles(data_list, 10)

def listener():
    global mc
    rospy.init_node("control_slider", anonymous=True)

    # ip = rospy.get_param("~ip", "192.168.1.159")
    # port = rospy.get_param("~port", 5001)
    # print (ip, port)
    # mc = ElephantRobot(ip, int(port))
    mc = Pro630Client("192.168.1.169", 9000)
    # START CLIENT,启动客户端
    res = mc.power_on()
    if res != 1:
        sys.exit(1)

    # cur_angles = mc.get_angles()
    # for i in range(len(cur_angles)):    cur_angles[i] = cur_angles[i] * math.pi / 180
    # print(f"get_angles={cur_angles}")

    rospy.Subscriber("joint_states", JointState, callback)

    # Create a publisher for joint_state topic
    # joint_state_publisher = rospy.Publisher("joint_state", JointState, queue_size=10)

    # Publish the initial joint state
    # joint_state_msg = JointState()
    # joint_state_msg.header.stamp = rospy.Time.now()
    # joint_state_msg.name = ["joint1_to_base","joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5"]
    # joint_state_msg.velocity = [0]
    # joint_state_msg.effort = []
    # joint_state_msg.position = cur_angles
    # joint_state_publisher.publish(joint_state_msg)

    # spin() simply keeps python from exiting until this node is stopped
    # spin()只是阻止python退出，直到该节点停止
    print ("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
