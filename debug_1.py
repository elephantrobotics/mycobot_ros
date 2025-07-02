#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import time, random, subprocess,rospy
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord
from pymycobot.myarmc import MyArmC

def get_joint_states():
    # 这个函数应该返回机械臂的实时关节状态
    # 这里只是一个示例，你需要根据你的实际情况来实现这个函数
    return [random.uniform(-180, 180) for _ in range(7)]

def talker():
    # 初始化节点
    rospy.init_node('joint_state_publisher')

    # 创建发布器，发布到'joint_states'话题，消息类型为JointState
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # 设置发布频率
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        # 创建JointState消息对象
        msg = JointState()

        # 获取机械臂的实时关节状态
        joint_states = get_joint_states()

        # 将关节状态赋值给消息对象
        msg.position = joint_states

        # 发布消息
        pub.publish(msg)

        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass