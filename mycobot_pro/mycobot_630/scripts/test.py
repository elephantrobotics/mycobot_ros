import time
import sys
from pymycobot.elephantrobot import ElephantRobot

import rospy
from math import pi
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot.mycobotpro630 import mycobot
import rosnode

mc = ElephantRobot('192.168.1.159', 5001, debug=True)

res = mc.start_client()
if res != "":
    print('quit')
    sys.exit(1)

print(mc.get_angles())



    
def linear_transform(x):
    # 两个已知数据点
    x1, y1 = -89.5, 0.022
    x2, y2 = 0, 0
    
    # 计算斜率
    m = (y2 - y1) / (x2 - x1)
    
    # 计算截距
    c = y1 - m * x1
    
    # 应用线性变换
    y = m * x + c
    
    return y

global mam
mam = mycobot('/dev/ttyACM0', debug=False)
angle = mam.get_joints_angle()
angle[1] += 20
# 初始化ROS节点
rospy.init_node("pub_data")
rosnode.kill_nodes('joint_state_publisher_gui')
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
    joint_state.name = ['joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2','joint4_to_joint3', 
                        'joint5_to_joint4', 'joint6_to_joint5',
                        'joint6output_to_gripper_base','gripper_controller']
    angles = mam.get_joints_angle()
    gripper_angle =  angles.pop(6)
    angle = [a/180*pi for a in angles]
    gripper_angle = linear_transform(gripper_angle)
    # gripper_angle = linear_transform(-107.66)
    angle.append(gripper_angle)
    
    joint_state.position = angle
    joint_state.effort = []
    joint_state.velocity = []
    # 发布消息
    pub.publish(joint_state)
    rospy.loginfo('消息成功发布')
    # 等待，允许其他节点处理
    rate.sleep()

