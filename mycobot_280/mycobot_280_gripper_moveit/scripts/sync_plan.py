#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot


def callback(data):
    # port = rospy.get_param("~port", "/dev/ttyUSB0")
    # baud = rospy.get_param("~baud", 115200)
    # # print(port, baud)
    # mc = MyCobot(port, baud)

    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(round(value, 3))
    data_list = data_list[:7]
    print("radians:%s" % data_list[:6])
    # t1 = time.time()
    mc.send_radians(data_list[:6], 80)
    # time.sleep(0.02)
    gripper_value = int(abs(-0.7 - data_list[6]) * 117)
    print("gripper_value:%s\n" % gripper_value)
    mc.set_gripper_value(gripper_value, 80)
    # t2 = time.time() - t1
    # print("cost time:", t2)
    # time.sleep(1)


def listener():
    global mc
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
