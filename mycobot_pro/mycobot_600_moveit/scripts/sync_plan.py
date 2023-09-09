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

old_list = []


def callback(data):
    """callback function,回调函数"""
    satrt_time=time.time()
    global old_list
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    print("position:", data.position)
    data_list = []
    for index, value in enumerate(data.position):
        value = value * 180 / math.pi
        data_list.append(value)
    print ("data", data_list)

    if not old_list:
        old_list = data_list
        mc.write_angles(data_list, 1999)
    elif old_list != data_list:
        old_list = data_list
        # if mc.check_running():
            # mc.task_stop()
            # time.sleep(0.05)
            
        mc.write_angles(data_list, 1999)

        end_time=time.time()
        print('loop_time:',end_time-satrt_time)

def listener():
    global mc
    rospy.init_node("control_slider", anonymous=True)

    ip = rospy.get_param("~ip", "192.168.1.159")
    print (ip)
    mc = ElephantRobot(ip, 5001)
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
