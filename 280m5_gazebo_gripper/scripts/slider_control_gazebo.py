#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slider_control.py
双模式滑块控制脚本：
  1: 滑块 -> Gazebo 控制器
  2: 滑块 -> 真实 MyCobot 机械臂
订阅 /joint_states（关节弧度），将关节角度解析并根据选择发布到 Gazebo，
或通过 pymycobot 发送到真实机械臂。
"""
import time
import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot.mycobot import MyCobot

# 全局变量
mc = None
mode = 2  # 默认使用真实机械臂模式
pub_arm = None
pub_gripper = None
last_angles = None  # 记录上次的角度，避免频繁发送相同命令

# 期望的关节名称
ARM_JOINTS = [
    "joint2_to_joint1",
    "joint3_to_joint2", 
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
GRIPPER_JOINT = "gripper_controller"

def is_mycobot_connected():
    """检查MyCobot是否连接"""
    global mc
    try:
        if mc is None:
            return False
        # 尝试获取当前角度来测试连接
        mc.get_angles()
        return True
    except Exception as e:
        rospy.logwarn(f"[slider_control] MyCobot连接检查失败: {e}")
        return False

def callback(msg: JointState):
    global last_angles
    
    # 从 JointState 中提取度数
    arm_deg = []
    grip_deg = None
    
    for name, pos in zip(msg.name, msg.position):
        deg = round(math.degrees(pos), 2)
        if name in ARM_JOINTS:
            arm_deg.append(deg)
        elif name == GRIPPER_JOINT:
            grip_deg = deg
    
    if len(arm_deg) != len(ARM_JOINTS):
        rospy.logerr(f"[slider_control] 期待 {len(ARM_JOINTS)} 个臂关节，实际收到 {len(arm_deg)} 个")
        return
        
    if grip_deg is None:
        rospy.logwarn("[slider_control] 未收到 gripper_controller 角度")
        grip_deg = 0.0  # 设置默认值
    
    if mode == 1:
        # Gazebo 模式：发布 JointTrajectory (暂时保留)
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(d) for d in arm_deg]
        pt.time_from_start = rospy.Duration(0.2)
        traj.points = [pt]
        pub_arm.publish(traj)
        
        traj_g = JointTrajectory()
        traj_g.header.stamp = rospy.Time.now()
        traj_g.joint_names = [GRIPPER_JOINT]
        ptg = JointTrajectoryPoint()
        ptg.positions = [math.radians(grip_deg)]
        ptg.time_from_start = rospy.Duration(0.2)
        traj_g.points = [ptg]
        pub_gripper.publish(traj_g)
        
    elif mode == 2:
        # 真实机械臂模式：通过 pymycobot
        if not is_mycobot_connected():
            rospy.logerr("[slider_control] MyCobot未连接，跳过控制命令")
            return
            
        # 检查角度变化，避免发送相同的命令
        if last_angles is not None:
            angle_diff = sum(abs(a - b) for a, b in zip(arm_deg, last_angles))
            if angle_diff < 1.0:  # 如果角度变化小于1度，跳过
                return
        
        # 发送角度命令
        try:
            rospy.loginfo(f"[slider_control] 发送角度: {arm_deg}")
            mc.send_angles(arm_deg, 30)  # 30°/s 的速度
            last_angles = arm_deg.copy()
        except Exception as e:
            rospy.logerr(f"[slider_control] 发送角度命令失败: {e}")
            
        # 夹爪控制 - 使用具体的开合度值，更明显的开合
        try:
            if grip_deg > 10:  # 大于10度时完全打开夹爪
                # 使用set_gripper_value设置具体开合度，100=完全打开
                mc.set_gripper_value(100, 70)
                rospy.loginfo(f"[slider_control] 夹爪完全打开 (角度: {grip_deg:.1f}°, 开合度: 100)")
            elif grip_deg < -10:  # 小于-10度时完全关闭夹爪
                # 0=完全关闭
                mc.set_gripper_value(0, 70)
                rospy.loginfo(f"[slider_control] 夹爪完全关闭 (角度: {grip_deg:.1f}°, 开合度: 0)")
            else:
                # 在-10到10度之间时，根据角度设置中间开合度
                # 将-10到10度映射到0到100的开合度
                gripper_openness = int((grip_deg + 10) * 100 / 20)
                gripper_openness = max(0, min(100, gripper_openness))
                mc.set_gripper_value(gripper_openness, 70)
                rospy.loginfo(f"[slider_control] 夹爪中间状态 (角度: {grip_deg:.1f}°, 开合度: {gripper_openness})")
                
            # 添加延时让夹爪有时间移动
            time.sleep(0.1)
                
            # 获取夹爪当前值用于调试
            try:
                gripper_value = mc.get_gripper_value()
                rospy.logdebug(f"[slider_control] 夹爪当前值: {gripper_value}")
            except:
                pass  # 忽略获取夹爪值的错误
                
        except Exception as e:
            rospy.logwarn(f"[slider_control] 夹爪控制失败: {e}")
            # 如果是超限位置错误，给出友好提示
            if "超限" in str(e) or "limit" in str(e).lower():
                rospy.logwarn("[slider_control] 夹爪已到超限位置，无法继续向前")
            elif "已到超限位置" in str(e):
                rospy.logwarn("[slider_control] 夹爪已到超限位置，无法继续向前")
            
        # 添加小延时避免命令发送过快
        time.sleep(0.01)

def initialize_mycobot():
    """初始化MyCobot连接"""
    global mc
    
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    
    rospy.loginfo(f"[slider_control] 尝试连接MyCobot: {port} @ {baud}")
    
    try:
        mc = MyCobot(port, baud)
        time.sleep(2.0)  # 增加等待时间确保连接稳定
        
        # 测试连接
        current_angles = mc.get_angles()
        rospy.loginfo(f"[slider_control] MyCobot连接成功，当前角度: {current_angles}")
        
        # 释放所有舵机，让机械臂可以被手动移动或接受命令
        mc.release_all_servos()
        time.sleep(0.5)
        
        return True
        
    except Exception as e:
        rospy.logerr(f"[slider_control] MyCobot初始化失败: {e}")
        rospy.logerr("[slider_control] 请检查:")
        rospy.logerr(f"  1. 设备是否连接到 {port}")
        rospy.logerr(f"  2. 波特率是否正确 ({baud})")
        rospy.logerr("  3. 用户是否有设备访问权限")
        rospy.logerr("  4. 设备是否被其他程序占用")
        return False

def main():
    global mc, mode, pub_arm, pub_gripper
    
    rospy.init_node("slider_control", anonymous=True)
    
    # 启动时选择模式
    print("\nSelect control mode:")
    print("  1: Slider → Gazebo")
    print("  2: Slider → Real MyCobot")
    inp = input("Enter 1 or 2 (default 2): ").strip()
    
    if inp == "1":
        mode = 1
    else:
        mode = 2
    
    rospy.loginfo(f"[slider_control] 工作模式: {'Gazebo' if mode==1 else 'Real Robot'}")
    
    # 初始化发布器（保留Gazebo支持）
    pub_arm = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
    pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
    
    if mode == 2:
        # 初始化真实机械臂
        if not initialize_mycobot():
            rospy.logerr("[slider_control] MyCobot初始化失败，退出程序")
            return
    
    # 订阅滑块发来的关节状态
    rospy.Subscriber("/joint_states", JointState, callback)
    
    rospy.loginfo("[slider_control] 节点启动成功，等待 /joint_states 消息...")
    rospy.loginfo("[slider_control] 请在RViz或其他工具中移动滑块来控制机械臂")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[slider_control] 收到中断信号，正在关闭...")
    finally:
        if mc is not None:
            try:
                mc.release_all_servos()
                rospy.loginfo("[slider_control] 已释放所有舵机")
            except:
                pass

if __name__ == "__main__":
    main()
