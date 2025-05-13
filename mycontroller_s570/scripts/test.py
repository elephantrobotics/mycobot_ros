#!/usr/bin/env python

"""
This package need `pymycobot`.
This file for test the API if right.

Just can run in Linux.
"""
from exoskeleton_api import Exoskeleton, ExoskeletonSocket
import rospy
from math import pi
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import rosnode
import os
import subprocess

os.system("sudo chmod 777 /dev/ttyACM*")

obj = Exoskeleton(port="/dev/ttyACM0")

def shutdown_ros_node(node_name):
    try:
        subprocess.run(['rosnode', 'kill', node_name])
        print(f"Node {node_name} has been shutdown.")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

# 初始化ROS节点
rospy.init_node("pub_data")
# rosnode.kill_nodes('/joint_state_publisher_gui')
shutdown_ros_node('joint_state_publisher_gui')
rospy.loginfo("已成功杀死节点")
# 创建发布者
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
# 设置发布频率
rate = rospy.Rate(100) # 100Hz
# 消息实例
joint_state = JointState()
# 发布消息
while not rospy.is_shutdown():
    joint_state.header = Header()
    # 填充消息内容，例如关节名称、位置、速度和力
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['joint1', 'joint2', 'joint3','joint4', 'joint5', 'joint6','joint7','joint8','joint9','joint10',
                        'joint11','joint12','joint13','joint14']
    l_angle = obj.get_arm_data(1)
    l_angle = l_angle[:7]
    l_angle = [int(x) for x in l_angle]
    print("l:",l_angle)
    r_angle = obj.get_arm_data(2)
    r_angle = r_angle[:7]
    r_angle = [int(y) for y in r_angle]
    print("r:",r_angle)
    angles = l_angle + r_angle
    angle = [a/180*pi for a in angles]
    
    joint_state.position = angle
    joint_state.effort = []
    joint_state.velocity = []
    # 发布消息
    pub.publish(joint_state)
    rospy.loginfo('消息成功发布')
    # 等待，允许其他节点处理
    rate.sleep()

