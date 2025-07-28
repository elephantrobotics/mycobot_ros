#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from socket import *
import math
import sys
import time
from multiprocessing import Lock
import numpy as np
import rospy
from sensor_msgs.msg import JointState

from pymycobot.elephantrobot import ElephantRobot

global mc
gripper_value = []



def linear_mapping(value, input_start, input_end, output_start, output_end):
    """
    Maps a value from one range to another using a linear transformation.

    Args:
        value (float): The input value to be mapped.
        input_start (float): The start of the input range.
        input_end (float): The end of the input range.
        output_start (float): The start of the output range.
        output_end (float): The end of the output range.

    Returns:
        float: The mapped value in the output range.
    """
    # Calculate the slope of the linear function
    slope = (output_end - output_start) / (input_end - input_start)
    # Calculate the mapped value
    mapped_value = output_start + slope * (value - input_start)
    return mapped_value


def callback(data):
    """callback function,回调函数"""
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)

    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)

    rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
    # mc.write_angles(data_list, 800)
    
    list_i = []
    for i in data_list[:6]:
        list_i.append(i)
    mc.write_angles(list_i,800)
    
    list_j = []
    for j in data_list[6:]:
        list_j.append(j)
    # list_j = [int(a) for a in list_j]
    
    mc.set_gripper_mode(0)
    # Define the input and output ranges
    input_start = 17
    input_end = -40
    output_start = 100
    output_end = 0
    num = [linear_mapping(value, input_start, input_end, output_start, output_end) for value in list_j]
    num2 = [int(i) for i in num]
    num2 = num2[1]
    mc.set_gripper_value(num2,100)
    rospy.loginfo(num2)


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
