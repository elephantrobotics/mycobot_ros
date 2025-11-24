#!/usr/bin/env python3
# encoding:utf-8
import time
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_pro450_communication.srv import GetAngles, GetGripperValue


def talker():
    rospy.loginfo("start ...")
    rospy.init_node("real_listener_gripper", anonymous=True)
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    rate = rospy.Rate(30)  # 30hz

    # pub joint state
    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_controller"]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    # waiting util server `get_joint_angles` enable
    rospy.loginfo("wait service")
    rospy.wait_for_service("get_joint_angles")
    rospy.wait_for_service("get_gripper_angle")
    func = rospy.ServiceProxy("get_joint_angles", GetAngles)
    gripper_service = rospy.ServiceProxy("get_gripper_angle", GetGripperValue)

    rospy.loginfo("start loop ...")
    gripper_value = 0
    while not rospy.is_shutdown():
        # get real angles from server
        res = func()
        gripper_res = gripper_service()
        if res.joint_1 == res.joint_2 == res.joint_3 == 0.0:
            continue
        # print('gripper_res.gripper_angle:', gripper_res.gripper_angle)
        if gripper_res.gripper_angle != -1:
            gripper_value = gripper_res.gripper_angle
        else:
            gripper_value = 1
            
        radians_list = [
            res.joint_1 * (math.pi / 180),
            res.joint_2 * (math.pi / 180),
            res.joint_3 * (math.pi / 180),
            res.joint_4 * (math.pi / 180),
            res.joint_5 * (math.pi / 180),
            res.joint_6 * (math.pi / 180),
            gripper_value * (math.pi / 180),
        ]
        # rospy.loginfo("res: {}".format(radians_list))

        # publish angles
        joint_state_send.header.stamp = rospy.Time.now()
        joint_state_send.position = radians_list
        pub.publish(joint_state_send)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
