#!/usr/bin/env python3
# encoding:utf-8
# 读取机器人关节角数据，发送给RVIZ

from pymycobot import MyArmM
import rospy
from math import pi
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import subprocess
# 1：[-174, 167]
# 2:[-92, 93]
# 3:[-91,103]
# 4:[-170, 170]
# 5:[-96,89]
# 6:[-170,170]
def shutdown_ros_node(node_name):
    try:
        subprocess.run(['rosnode', 'kill', node_name])
        print(f"Node {node_name} has been shutdown.")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

global mam
mam = MyArmM('/dev/ttyACM1', debug=False)
# 1 3 4 6
# 放松关节
for i in range(8):
    mam.set_servo_enabled(i, 0)
    time.sleep(0.2)

# 初始化ROS节点
rospy.init_node("pub_data")
shutdown_ros_node('joint_state_publisher_gui')
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
    joint_state.name = ['joint1', 'joint2', 'joint3','joint4', 'joint5', 'joint6', 'gripper']
    angles = mam.get_joints_angle()
    gripper_angle = angles.pop(6)
    gripper_angle /= -3500
    # angle:弧度 angles:角度
    angle = [a/180*pi for a in angles]
    angle.append(gripper_angle)
    print(angle)
    joint_state.position = angle
    joint_state.effort = []
    joint_state.velocity = []
    # 发布消息
    pub.publish(joint_state)
    rospy.loginfo('消息成功发布')
    # 等待，允许其他节点处理
    rate.sleep()


# print(mam.get_servos_encoder())
