#!/usr/bin/env python2
import time, subprocess
import rospy
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        # if index != 2:
        #     value *= -1
        data_list.append(value)

    mc.send_radians(data_list, 80)
    
def listener():
    rospy.init_node('mycobot_reciver', anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    port = subprocess.check_output(['echo -n /dev/ttyUSB*'], 
                                    shell=True)
    mc = MyCobot(port)
    listener()