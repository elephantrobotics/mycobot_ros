#!/usr/bin/env python2
# encoding=utf-8
import time
import rospy
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot
# from pymycobot.mypalletizer import MyPalletizer


mc = None


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        # if index != 2:
        #     value *= -1
        data_list.append(value)
    # print("data_list:",data_list)
    mc.send_radians(data_list, 80)


def listener():
    global mc
    rospy.init_node("mypal_reciver", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped 
    # spin()只是阻止python退出，直到该节点停止
    rospy.spin()


if __name__ == "__main__":
    listener()
