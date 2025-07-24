#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial port string. Defaults is auto-detected or '/dev/ttyUSB0'
    baud: serial port baudrate. Defaults is 115200.
"""

import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import serial.tools.list_ports
from pymycobot import PI_PORT, PI_BAUD               # Raspberry Pi 版初始化
from pymycobot.mycobot import MyCobot
from pymycobot import MyCobot280                     # Raspberry Pi 版 MyCobot

# 保留原 mcs 实例用于获取 gripper 角度
mcs = None
mc = None
pub_arm_command = None
pub_gripper_command = None

# 上一次有效映射值（用于丢弃无效 254/255 读数时"冻结"夹爪位置）
last_mapped_gripper = None

# 映射常量
MYCOBOT_MIN_ANGLE = 3
MYCOBOT_MAX_ANGLE = 91
GAZEBO_MIN_POSITION = -0.68
GAZEBO_MAX_POSITION = 0.15

def find_available_port():
    """自动检测可用串口，优先选择USB转串口设备"""
    ports = serial.tools.list_ports.comports()
    
    # 优先级关键词列表（从高到低）
    priority_keywords = ['ACM', 'USB', 'Arduino', 'CH340', 'CP210', 'FTDI']
    
    # 按优先级查找串口
    for keyword in priority_keywords:
        for port in ports:
            if (keyword in port.device.upper() or 
                keyword in port.description.upper() or 
                keyword in str(port.hwid).upper()):
                rospy.loginfo(f"找到优先串口: {port.device} ({port.description})")
                return port.device
    
    # 如果没找到优先设备，返回第一个可用串口
    if ports:
        selected_port = ports[0].device
        rospy.loginfo(f"使用第一个可用串口: {selected_port} ({ports[0].description})")
        return selected_port
    
    # 没有找到任何串口
    rospy.logwarn("未找到任何可用串口，使用默认值")
    return "/dev/ttyUSB0"

def list_available_ports():
    """列出所有可用串口信息"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        rospy.loginfo("没有找到可用串口")
        return
    
    rospy.loginfo("=== 可用串口列表 ===")
    for i, port in enumerate(ports):
        rospy.loginfo(f"{i+1}. 设备: {port.device}")
        rospy.loginfo(f"   描述: {port.description}")
        rospy.loginfo(f"   硬件ID: {port.hwid}")
        rospy.loginfo("   ---")

def callback(data):
    global last_mapped_gripper

    # --- ARM 控制逻辑保持不变 ---
    expected_arm_joint_names = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6"
    ]
    expected_gripper_joint_names = ["gripper_controller"]

    # 获取真实机械臂关节与夹爪原始数据
    try:
        angles = mc.get_angles()
        gripper_ang = mcs.get_gripper_value()
    except Exception as e:
        rospy.logerr(f"获取机械臂数据失败: {e}")
        return

    # 如果读数为 254 或 255，则打印 0；否则打印真实角度
    if gripper_ang in (254, 255):
        print(0)
    else:
        print(gripper_ang)

    # arm 角度转换成弧度列表
    data_list = [v * (math.pi / 180) for v in angles]

    # 验证 joint_states 长度
    filtered_arm_positions = []
    filtered_gripper_positions = []
    for name, pos in zip(data.name, data.position):
        if name in expected_arm_joint_names:
            filtered_arm_positions.append(round(math.degrees(pos), 2))
        elif name in expected_gripper_joint_names:
            filtered_gripper_positions.append(round(math.degrees(pos), 2))

    if len(filtered_arm_positions) != len(expected_arm_joint_names):
        rospy.logerr(f"Received {len(filtered_arm_positions)} arm positions, but expected {len(expected_arm_joint_names)}.")
        return
    if len(filtered_gripper_positions) != len(expected_gripper_joint_names):
        rospy.logerr(f"Received {len(filtered_gripper_positions)} gripper positions, but expected {len(expected_gripper_joint_names)}.")
        return

    # 发布 arm 轨迹
    arm_traj = JointTrajectory()
    arm_traj.header.stamp = rospy.Time.now()
    arm_traj.joint_names = expected_arm_joint_names
    pt = JointTrajectoryPoint()
    pt.positions = data_list
    pt.velocities = [0.0] * len(expected_arm_joint_names)
    pt.accelerations = [0.0] * len(expected_arm_joint_names)
    pt.time_from_start = rospy.Duration(0.5)
    arm_traj.points.append(pt)
    pub_arm_command.publish(arm_traj)

    # --- GRIPPER 控制：丢弃 254/255 读数并"冻结"位置 ---
    if gripper_ang in (254, 255):
        # 无效读数，保持上一次有效值或初始化到 fully closed
        if last_mapped_gripper is None:
            mapped = GAZEBO_MIN_POSITION
        else:
            mapped = last_mapped_gripper
    else:
        # 正常线性映射
        mapped = ((gripper_ang - MYCOBOT_MIN_ANGLE) /
                  (MYCOBOT_MAX_ANGLE - MYCOBOT_MIN_ANGLE)) * \
                 (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION) + GAZEBO_MIN_POSITION
        # clamp 范围
        mapped = max(GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, mapped))
        # 更新有效映射
        last_mapped_gripper = mapped

    # 发布 gripper 轨迹
    gripper_traj = JointTrajectory()
    gripper_traj.header.stamp = rospy.Time.now()
    gripper_traj.joint_names = expected_gripper_joint_names
    gp = JointTrajectoryPoint()
    gp.positions = [mapped]
    gp.velocities = [0.0]
    gp.accelerations = [0.0]
    gp.time_from_start = rospy.Duration(0.5)
    gripper_traj.points.append(gp)
    pub_gripper_command.publish(gripper_traj)


def listener():
    global mc, mcs, pub_arm_command, pub_gripper_command

    rospy.init_node("control_slider", anonymous=True)
    
    # 显示所有可用串口
    list_available_ports()
    
    # 获取串口参数，优先使用ROS参数，其次自动检测
    port = rospy.get_param("~port", find_available_port())
    baud = rospy.get_param("~baud", 115200)
    
    rospy.loginfo(f"使用串口: {port}, 波特率: {baud}")

    try:
        # 初始化真实机械臂连接
        mc = MyCobot(port, baud)
        mc.release_all_servos()
        
        # 初始化夹爪连接（使用相同端口）
        mcs = MyCobot280(port, baud)
        
        time.sleep(0.1)
        rospy.loginfo("机械臂连接成功")
        
    except Exception as e:
        rospy.logerr(f"机械臂连接失败: {e}")
        rospy.logerr("请检查:")
        rospy.logerr("1. 机械臂是否正确连接到电脑")
        rospy.logerr("2. 串口权限是否正确 (sudo chmod 666 /dev/ttyACM0)")
        rospy.logerr("3. 是否有其他程序占用串口")
        return

    # 初始化 publishers & subscriber
    pub_arm_command = rospy.Publisher("/arm_controller/command",
                                      JointTrajectory, queue_size=10)
    pub_gripper_command = rospy.Publisher("/gripper_controller/command",
                                          JointTrajectory, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, callback)

    rospy.loginfo("Slider control node started.")
    rospy.spin()


if __name__ == "__main__":
    listener()
