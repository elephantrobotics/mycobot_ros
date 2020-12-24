#!/usr/bin/env python3
# from std_msgs.msg import String
import time

import rospy
from sensor_msgs.msg import JointState

from pythonAPI.mycobot import MyCobot


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    # print(data.position)
    data_list = []
    for index, value in enumerate(data.position):
        if index != 2:
            value *= -1
        data_list.append(value)

    mc.send_angles_by_radian(data_list, 80)
    # time.sleep(0.5)
    
def listener():
    rospy.init_node('control_slider', anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    mc = MyCobot()
    listener()
