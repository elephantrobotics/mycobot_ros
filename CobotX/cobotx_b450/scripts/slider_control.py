#!/usr/bin/env python3

"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyAMA1'
    baud: serial prot baudrate. Defaults is 115200.
"""
import math
import time
import rospy
from sensor_msgs.msg import JointState

from pymycobot.cobotx import CobotX

# left arm port
cx1 = None

# right arm port
cx2 = None


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    rounded_data_tuple = tuple(round(value, 2) for value in data.position)
    # print(rounded_data_tuple)
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)
    print('data_list:', data_list)
    left_arm = data_list[:7]
    right_arm = data_list[7:-3]
    middle_arm = data_list[-3:]
    
    print('left_arm:', left_arm)
    print('right_arm:', right_arm)
    print('middle_arm:', middle_arm)
    
    cx1.send_angles(left_arm, 50)
    time.sleep(0.02)
    cx2.send_angles(right_arm, 50)
    time.sleep(0.02)
    cx2.send_angle(11, middle_arm[0], 50)
    time.sleep(0.02)
    cx2.send_angle(12, middle_arm[1], 50)
    time.sleep(0.02)
    cx2.send_angle(13, middle_arm[2], 50)
    time.sleep(0.02)
    # mc.send_radians(data_list, 80)
    # mc.send_angles(data_list, 80)
    # time.sleep(0.5)


def listener():
    global cx1, cx2
    rospy.init_node("control_slider", anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)
    port1 = rospy.get_param("~port1", "/dev/ttyS0")
    port2 = rospy.get_param("~port2", "/dev/ttyTHS1")
    baud = rospy.get_param("~baud", 115200)
    print(port1, baud)
    print(port2, baud)
    cx1 = CobotX(port1, baud)
    cx2 = CobotX(port2, baud)

    # spin() simply keeps python from exiting until this node is stopped
    # spin()只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
