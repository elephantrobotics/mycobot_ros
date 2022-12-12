#!/usr/bin/env python3
# encoding:utf-8
"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyTHS1'
    baud: serial prot baudrate. Defaults is 1000000.
"""

import rospy
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot
from pymycobot.mycobotsocket import MyCobotSocket


mc = None


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    print(data.position)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(value)

    mc.send_radians(data_list, 80)
    # time.sleep(0.5)


def listener():
    global mc
    rospy.init_node("control_slider", anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)
    port = rospy.get_param("~port", "/dev/ttyTHS1")
    baud = rospy.get_param("~baud", 1000000)
    print(port, baud)
    mc = MyCobot(port, baud)
    # ip=rospy.get_param("~ip",'192.168.125.226')
    # port=rospy.get_param("~port",9000)
    # print(ip,port)
    # ms=MyCobotSocket(ip,port)
    # ms.connect()

    # spin() simply keeps python from exiting until this node is stopped
    # spin()只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
