#!/usr/bin/env python3
# encoding:utf-8
from pymycobot import MyArmM, MyArmC
from sensor_msgs.msg import JointState
import rospy
from std_msgs.msg import Header
from math import pi
import subprocess
import sys
import time
import datetime
import copy
def shutdown_ros_node(node_name):
    try:
        subprocess.run(['rosnode', 'kill', node_name])
        print(f"Node {node_name} has been shutdown.")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")


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


def set_angle(mam, angle, speed=40):
    # 弧度转角度
    # angle = [int(a*180/pi) for a in angles]
    # angle.append(0)
    # joint_id = {0:1, 1:2, 2:3, 3:4, 4:5, 5:6}
    # joint_id = [0,1,2,3,4,5,6]
    mam.set_joints_angle(angle, speed)
    
    # for i in range(6):
    #     # if joint_id[i] == 2 or joint_id[i] == 5:
    #     #     angle[i] *= -1
    #     mam.set_joint_angle(joint_id[i], -angle[i], 50)
def main():
    # 初始化ROS节点
    rospy.init_node("combined_control", anonymous=True)
    # 关闭节点
    shutdown_ros_node('myarm_m/joint_state_publisher_gui')
    shutdown_ros_node('myarm_c650/joint_state_publisher_gui')
    # 发布对象
    pub_m = rospy.Publisher('myarm_m/joint_states', JointState, queue_size=10)
    pub_c = rospy.Publisher('myarm_c650/joint_states', JointState, queue_size=10)
    # 设置发布频率
    rate = rospy.Rate(50)
    # 消息实例
    joint_state = JointState()
    # 初始化api对象
    myarm_m = MyArmM('/dev/ttyACM1', debug=False)
    myarm_c = MyArmC('/dev/ttyACM0', debug=False)
    # 使能
    for i in range(8):
        myarm_m.set_servo_enabled(i, 1)
        time.sleep(0.2)

    while not rospy.is_shutdown():
        joint_state.header = Header()
        # 填充消息内容，例如关节名称、位置、速度和力
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['joint1', 'joint2', 'joint3','joint4', 'joint5', 'joint6','gripper']
        anglesc = myarm_c.get_joints_angle()
        anglesm = copy.deepcopy(anglesc)
        
        gripper_angle_c_real = anglesc.pop(6)
        angle_c = [a/180*pi for a in anglesc]
        gripper_angle_c_sim = linear_transform(gripper_angle_c_real)
        angle_c.append(gripper_angle_c_sim)
        
        gripper_angle_c_real = anglesm.pop(6) # 原来的夹角
        gripper_angle_m_sim = gripper_angle_c_sim/0.022*0.0345
        angle_m = [a*pi/180 for a in anglesm]
        angle_m.append(gripper_angle_m_sim)
        # gripper_angle_m /= -3500
        # angle:弧度 angles:角度
        
        angle_m[2]*=-1
        joint_state.position = angle_m
        joint_state.effort = []
        joint_state.velocity = []
        # 发布消息
        pub_m.publish(joint_state)
        current_time1 = datetime.datetime.now()
        joint_state.position = angle_c
        pub_c.publish(joint_state)
        current_time2 = datetime.datetime.now()
        gripper_angle_m_sim = angle_m.pop(6)
        gripper_angle_m_real = gripper_angle_m_sim*(-3500)
        angle_m = [a*180/pi for a in angle_m]
        angle_m.append(gripper_angle_m_real)
        myarm_m.set_joints_angle(angle_m, 30)
        current_time = current_time2-current_time1
        print(angle_m)
        rospy.loginfo('消息成功发布')
        rospy.loginfo(current_time)
        # 等待，允许其他节点处理
        rate.sleep()
        



if __name__ == '__main__':
    main()