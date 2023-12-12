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

from pymycobot.mercury import Mercury

# left arm port
l = None

# right arm port
r = None


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)

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
    
    l.send_angles(left_arm, 25)
    time.sleep(0.02)
    r.send_angles(right_arm, 25)
    time.sleep(0.02)
    r.send_angle(11, middle_arm[0], 25)
    time.sleep(0.02)
    r.send_angle(12, middle_arm[1], 25)
    time.sleep(0.02)
    r.send_angle(13, middle_arm[2], 25)
    time.sleep(0.02)


def listener():
    global l, r
    rospy.init_node("control_slider", anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)
    port1 = rospy.get_param("~port1", "/dev/ttyTHS1")
    port2 = rospy.get_param("~port2", "/dev/ttyS0")
    baud = rospy.get_param("~baud", 115200)
    print(port1, baud)
    print(port2, baud)
    l = Mercury(port1, baud)
    r = Mercury(port2, baud)
    time.sleep(0.05)
    l.set_free_mode(1)
    r.set_free_mode(1)
    time.sleep(0.05)
    # spin() simply keeps python from exiting until this node is stopped
    # spin()只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
