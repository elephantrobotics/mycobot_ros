#!/usr/bin/env python

"""
This package need `pymycobot`.
This file for test the API if right.

Just can run in Linux.
"""

import rospy
from math import pi
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot.myarmc import MyArmC
import rosnode
    

global mam
mam = MyArmC('/dev/ttyACM0', debug=False)
angle = mam.get_joints_angle()
angle[1] += 20
# 初始化ROS节点
rospy.init_node("pub_data")
rosnode.kill_nodes('joint_state_publisher_gui')
rospy.loginfo("已成功杀死节点")
# 创建发布者
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
# 设置发布频率
rate = rospy.Rate(50) # 50Hz
# 消息实例
joint_state = JointState()
# 发布消息
while not rospy.is_shutdown():
    joint_state.header = Header()
    # 填充消息内容，例如关节名称、位置、速度和力
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2','joint4_to_joint3', 'joint5_to_joint4', 'endeffector_to_joint5']
    angles = mam.get_joints_angle()
    angles.pop(6)
    angle = [a/180*pi for a in angles]

    joint_state.position = angle
    joint_state.effort = []
    joint_state.velocity = []
    # 发布消息
    pub.publish(joint_state)
    rospy.loginfo('消息成功发布')
    # 等待，允许其他节点处理
    rate.sleep()

