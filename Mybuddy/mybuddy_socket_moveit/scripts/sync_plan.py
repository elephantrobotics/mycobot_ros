#!/usr/bin/env python3
import time
import rospy
import os
from sensor_msgs.msg import JointState
from pymycobot.mybuddysocket import MyBuddySocket
import math

mb = None


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(value)

    data_list1 = data_list[:6]
    data_list2 = data_list[6:-1]
    data_list3 = list(data_list[-1:])

    print("left_arm: %s" % data_list1)
    print("right_arm: %s" % data_list2)
    print("waist: %s" % data_list3)

    print("\n")
    mb.send_radians(1,data_list1, 50)
    time.sleep(0.02)
    mb.send_radians(2,data_list2, 50)
    time.sleep(0.02)
    mb.send_angle(3, 1, data_list3[0]* (180 / math.pi), 10)
    time.sleep(0.02)

def listener():
    global mb
    rospy.init_node("mybuddy_reciver", anonymous=True)
    ip = rospy.get_param("~ip", "192.168.123.219")
    port = rospy.get_param("~port", 9000)
    print(ip, port)
    mb = MyBuddySocket(ip, port)
    mb.connect(serialport="/dev/ttyACM0", baudrate="115200")
    
    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    listener()
