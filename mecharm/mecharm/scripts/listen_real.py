#!/usr/bin/env python2
# -*- coding:utf-8 -*-
# license removed for brevity
import time
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_communication.srv import GetAngles


def talker():
    rospy.loginfo("start ...")
    rospy.init_node("real_listener", anonymous=True)
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    rate = rospy.Rate(30)  # 30hz

    # pub joint state 发布关节状态
    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = [
        "joint1_to_base",
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
    ]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    # waiting util server `get_joint_angles` enable.
    rospy.loginfo("wait service")
    rospy.wait_for_service("get_joint_angles")
    func = rospy.ServiceProxy("get_joint_angles", GetAngles)

    rospy.loginfo("start loop ...")
    while not rospy.is_shutdown():
        # get real angles from server.
        #  从服务获取真实的角度
        res = func()
        if res.joint_1 == res.joint_2 == res.joint_3 == 0.0:
            continue
        radians_list = [
            res.joint_1 * (math.pi / 180),
            res.joint_2 * (math.pi / 180),
            res.joint_3 * (math.pi / 180),
            res.joint_4 * (math.pi / 180),
            res.joint_5 * (math.pi / 180),
            res.joint_6 * (math.pi / 180),

        ]
        rospy.loginfo("res: {}".format(radians_list))

        # publish angles. 发布角度
        joint_state_send.header.stamp = rospy.Time.now()
        joint_state_send.position = radians_list
        pub.publish(joint_state_send)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
