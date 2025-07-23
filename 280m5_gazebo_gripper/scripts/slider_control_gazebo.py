#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slider_control.py - 优化版本
双模式滑块控制脚本：
  1: 滑块 -> Gazebo 控制器  
  2: 滑块 -> 真实 MyCobot 机械臂
使用异步执行和频率控制优化性能，减少卡顿
"""
import time
import math
import threading
import queue
from collections import deque
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot.mycobot import MyCobot

# 全局变量
mc = None
mode = 2
pub_arm = None
pub_gripper = None

# 优化参数
ANGLE_THRESHOLD = 3.0           # 角度变化阈值(度)
GRIPPER_THRESHOLD = 5.0         # 夹爪角度变化阈值(度)  
MAX_COMMAND_RATE = 10.0         # 最大命令频率(Hz)
COMMAND_QUEUE_SIZE = 5          # 命令队列大小

# 安全角度限制 (度)
JOINT_LIMITS = [
    (-180, 180),  # joint1
    (-180, 180),  # joint2
    (-180, 180),  # joint3
    (-180, 180),  # joint4
    (-180, 180),  # joint5
    (-180, 180),  # joint6
]

GRIPPER_LIMITS = (-30, 30)  # 夹爪角度限制

# 状态记录
last_angles = None
last_gripper_angle = None
last_command_time = 0
command_queue = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
is_executing = False

# 超限警告控制
last_warning_time = {}  # 每个关节的最后警告时间
WARNING_INTERVAL = 3.0  # 警告间隔(秒)

# 统计信息
stats = {
    'total_messages': 0,
    'commands_sent': 0,
    'commands_skipped': 0,
    'limit_violations': 0,
    'errors': 0
}

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

class RobotCommand:
    """机器人命令类"""
    def __init__(self, cmd_type, data, timestamp=None):
        self.type = cmd_type  # 'angles' or 'gripper'
        self.data = data
        self.timestamp = timestamp or time.time()

def is_mycobot_connected():
    """检查MyCobot是否连接"""
    global mc
    try:
        if mc is None:
            return False
        mc.get_angles()
        return True
    except Exception as e:
        return False

def check_angle_limits(angles, gripper_angle):
    """检查角度是否在安全范围内"""
    global last_warning_time, stats
    
    current_time = time.time()
    violations = []
    
    # 检查关节角度限制
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        if angle < min_limit or angle > max_limit:
            joint_key = f"joint{i+1}"
            
            # 控制警告频率
            if joint_key not in last_warning_time or current_time - last_warning_time[joint_key] > WARNING_INTERVAL:
                violations.append(f"关节{i+1}: {angle:.1f}° (限制: {min_limit}°~{max_limit}°)")
                last_warning_time[joint_key] = current_time
                stats['limit_violations'] += 1
    
    # 检查夹爪角度限制
    min_grip, max_grip = GRIPPER_LIMITS
    if gripper_angle < min_grip or gripper_angle > max_grip:
        gripper_key = "gripper"
        
        if gripper_key not in last_warning_time or current_time - last_warning_time[gripper_key] > WARNING_INTERVAL:
            violations.append(f"夹爪: {gripper_angle:.1f}° (限制: {min_grip}°~{max_grip}°)")
            last_warning_time[gripper_key] = current_time
            stats['limit_violations'] += 1
    
    # 打印警告信息
    if violations:
        rospy.logwarn(f"[slider_control] ⚠️  角度超限:")
        for violation in violations:
            rospy.logwarn(f"[slider_control]    {violation}")
    
    return len(violations) == 0

def clamp_angles(angles, gripper_angle):
    """将角度限制在安全范围内"""
    # 限制关节角度
    clamped_angles = []
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        clamped_angle = max(min_limit, min(max_limit, angle))
        clamped_angles.append(clamped_angle)
    
    # 限制夹爪角度
    min_grip, max_grip = GRIPPER_LIMITS
    clamped_gripper = max(min_grip, min(max_grip, gripper_angle))
    
    return clamped_angles, clamped_gripper

def calculate_angle_difference(angles1, angles2):
    """计算角度差异"""
    if angles1 is None or angles2 is None:
        return float('inf')
    return sum(abs(a - b) for a, b in zip(angles1, angles2))

def should_send_command(new_angles, new_gripper_angle):
    """判断是否应该发送命令"""
    global last_angles, last_gripper_angle, last_command_time
    
    current_time = time.time()
    
    # 频率限制
    if current_time - last_command_time < 1.0 / MAX_COMMAND_RATE:
        return False, "频率限制"
    
    # 角度变化检查
    angle_diff = calculate_angle_difference(new_angles, last_angles)
    gripper_diff = abs(new_gripper_angle - last_gripper_angle) if last_gripper_angle is not None else float('inf')
    
    if angle_diff < ANGLE_THRESHOLD and gripper_diff < GRIPPER_THRESHOLD:
        return False, f"角度变化太小 (臂:{angle_diff:.1f}°, 夹爪:{gripper_diff:.1f}°)"
    
    return True, "允许发送"

def add_command_to_queue(command):
    """添加命令到队列"""
    try:
        command_queue.put_nowait(command)
        return True
    except queue.Full:
        # 队列满时移除最旧命令
        try:
            old_command = command_queue.get_nowait()
            command_queue.put_nowait(command)
            return True
        except queue.Empty:
            return False

def command_executor():
    """异步命令执行线程"""
    global is_executing, last_angles, last_gripper_angle, last_command_time, stats
    
    while not rospy.is_shutdown():
        try:
            # 获取命令，超时1秒
            command = command_queue.get(timeout=1.0)
            
            if not is_mycobot_connected():
                rospy.logwarn("[slider_control] MyCobot未连接，跳过命令")
                stats['errors'] += 1
                continue
            
            is_executing = True
            
            try:
                if command.type == 'angles':
                    # 发送角度命令
                    mc.send_angles(command.data, 40)  # 提高速度
                    last_angles = command.data.copy()
                    rospy.logdebug(f"[slider_control] 发送角度: {command.data}")
                    
                elif command.type == 'gripper':
                    # 发送夹爪命令
                    gripper_angle = command.data
                    
                    # 简化夹爪控制逻辑
                    if gripper_angle > 10:
                        openness = 100
                    elif gripper_angle < -10:
                        openness = 0
                    else:
                        openness = int((gripper_angle + 10) * 5)  # 映射到0-100
                        openness = max(0, min(100, openness))
                    
                    mc.set_gripper_value(openness, 80)
                    last_gripper_angle = gripper_angle
                    rospy.logdebug(f"[slider_control] 夹爪: {gripper_angle:.1f}° -> {openness}%")
                
                stats['commands_sent'] += 1
                last_command_time = time.time()
                
            except Exception as e:
                rospy.logerr(f"[slider_control] 命令执行失败: {e}")
                stats['errors'] += 1
                
            finally:
                is_executing = False
                command_queue.task_done()
                
        except queue.Empty:
            continue
        except Exception as e:
            rospy.logerr(f"[slider_control] 命令执行器错误: {e}")
            is_executing = False

def callback(msg: JointState):
    """优化的回调函数"""
    global stats
    
    stats['total_messages'] += 1
    
    # 快速解析关节数据
    arm_deg = [0.0] * len(ARM_JOINTS)
    grip_deg = 0.0
    
    name_to_deg = {name: math.degrees(pos) for name, pos in zip(msg.name, msg.position)}
    
    # 提取臂关节角度
    for i, joint_name in enumerate(ARM_JOINTS):
        if joint_name in name_to_deg:
            arm_deg[i] = round(name_to_deg[joint_name], 1)
    
    # 提取夹爪角度
    if GRIPPER_JOINT in name_to_deg:
        grip_deg = round(name_to_deg[GRIPPER_JOINT], 1)
    
    # 检查角度限制
    check_angle_limits(arm_deg, grip_deg)
    
    if mode == 1:
        # Gazebo模式 - 保持原有逻辑但加上限制检查
        # 为Gazebo模式也进行角度限制（可选）
        clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg)
        publish_to_gazebo(clamped_arm, clamped_grip)
        
    elif mode == 2:
        # 真实机械臂模式 - 异步处理
        should_send, reason = should_send_command(arm_deg, grip_deg)
        
        if should_send:
            # 对真实机械臂进行角度限制
            clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg)
            
            # 添加臂关节命令
            arm_command = RobotCommand('angles', clamped_arm)
            if add_command_to_queue(arm_command):
                # 添加夹爪命令（如果角度变化足够大）
                gripper_diff = abs(clamped_grip - last_gripper_angle) if last_gripper_angle is not None else float('inf')
                if gripper_diff >= GRIPPER_THRESHOLD:
                    gripper_command = RobotCommand('gripper', clamped_grip)
                    add_command_to_queue(gripper_command)
            else:
                stats['commands_skipped'] += 1
        else:
            stats['commands_skipped'] += 1
            rospy.logdebug(f"[slider_control] 跳过命令: {reason}")

def publish_to_gazebo(arm_deg, grip_deg):
    """发布到Gazebo"""
    global pub_arm, pub_gripper
    
    # 臂关节轨迹
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ARM_JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(d) for d in arm_deg]
    pt.time_from_start = rospy.Duration(0.1)
    traj.points = [pt]
    pub_arm.publish(traj)
    
    # 夹爪轨迹
    traj_g = JointTrajectory()
    traj_g.header.stamp = rospy.Time.now()
    traj_g.joint_names = [GRIPPER_JOINT]
    ptg = JointTrajectoryPoint()
    ptg.positions = [math.radians(grip_deg)]
    ptg.time_from_start = rospy.Duration(0.1)
    traj_g.points = [ptg]
    pub_gripper.publish(traj_g)

def initialize_mycobot():
    """初始化MyCobot连接"""
    global mc
    
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    
    rospy.loginfo(f"[slider_control] 连接MyCobot: {port} @ {baud}")
    
    try:
        mc = MyCobot(port, baud)
        time.sleep(2.0)
        
        # 测试连接
        current_angles = mc.get_angles()
        rospy.loginfo(f"[slider_control] MyCobot连接成功: {current_angles}")
        
        mc.release_all_servos()
        time.sleep(0.5)
        
        return True
        
    except Exception as e:
        rospy.logerr(f"[slider_control] MyCobot初始化失败: {e}")
        return False

def print_stats():
    """打印统计信息"""
    if stats['total_messages'] > 0:
        efficiency = (stats['commands_sent'] / stats['total_messages']) * 100
        rospy.loginfo(f"[slider_control] 统计: 消息:{stats['total_messages']}, "
                      f"发送:{stats['commands_sent']}, 跳过:{stats['commands_skipped']}, "
                      f"超限:{stats['limit_violations']}, 错误:{stats['errors']}, "
                      f"效率:{efficiency:.1f}%")

def main():
    global mc, mode, pub_arm, pub_gripper
    
    rospy.init_node("slider_control_optimized", anonymous=True)
    
    # 模式选择
    print("\nSelect control mode:")
    print("  1: Slider → Gazebo")
    print("  2: Slider → Real MyCobot (Optimized)")
    inp = input("Enter 1 or 2 (default 2): ").strip()
    
    mode = 1 if inp == "1" else 2
    
    rospy.loginfo(f"[slider_control] 模式: {'Gazebo' if mode==1 else 'Real Robot (优化版)'}")
    rospy.loginfo(f"[slider_control] 配置: 角度阈值={ANGLE_THRESHOLD}°, "
                  f"最大频率={MAX_COMMAND_RATE}Hz, 队列大小={COMMAND_QUEUE_SIZE}")
    rospy.loginfo(f"[slider_control] 安全限制: 关节±180°, 夹爪{GRIPPER_LIMITS[0]}°~{GRIPPER_LIMITS[1]}°")
    
    # 初始化发布器
    pub_arm = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
    pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1)
    
    if mode == 2:
        # 初始化真实机械臂
        if not initialize_mycobot():
            rospy.logerr("[slider_control] MyCobot初始化失败，退出")
            return
        
        # 启动命令执行线程
        executor_thread = threading.Thread(target=command_executor, daemon=True)
        executor_thread.start()
        rospy.loginfo("[slider_control] 异步命令执行器已启动")
    
    # 订阅关节状态
    rospy.Subscriber("/joint_states", JointState, callback, queue_size=1)
    
    rospy.loginfo("[slider_control] 节点启动成功，等待滑块输入...")
    rospy.loginfo("[slider_control] 💡 超限时会自动限制角度并显示警告")
    
    # 定期打印统计信息
    def stats_timer():
        while not rospy.is_shutdown():
            time.sleep(10.0)  # 每10秒打印一次
            print_stats()
    
    stats_thread = threading.Thread(target=stats_timer, daemon=True)
    stats_thread.start()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[slider_control] 收到中断信号，正在关闭...")
    finally:
        print_stats()  # 最终统计
        if mc is not None:
            try:
                mc.release_all_servos()
                rospy.loginfo("[slider_control] 已释放所有舵机")
            except:
                pass

if __name__ == "__main__":
    main()
