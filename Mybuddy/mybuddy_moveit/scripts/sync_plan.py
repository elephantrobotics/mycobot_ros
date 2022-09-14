#!/usr/bin/env python3
import time
import rospy
import os
from sensor_msgs.msg import JointState
from pymycobot.mybuddy import MyBuddy
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
    # mb.send_radians(3,data_list3, 50)
    # time.sleep(0.02)
    # mb.set_encoder(3,1,data_list3[0]*4096/(2*math.pi)+2048,1)
    mb.send_angle(3, 1, data_list3[0]* (180 / math.pi), 10)
    time.sleep(0.02)

def listener():
    global mb
    rospy.init_node("mybuddy_reciver", anonymous=True)
    port = rospy.get_param("~port", os.popen("ls /dev/ttyACM*").readline()[:-1])
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mb = MyBuddy(port, baud)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    listener()
