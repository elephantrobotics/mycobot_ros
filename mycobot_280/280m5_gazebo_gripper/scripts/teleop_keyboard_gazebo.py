#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import termios
import tty
import sys
import rospy
import math
import time
import threading
import queue
import serial
import serial.tools.list_ports
from pymycobot.mycobot import MyCobot
from pymycobot import MyCobot280, PI_PORT, PI_BAUD
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# --------------------------- 全局变量 ---------------------------
mc = None
mcs = MyCobot280(PI_PORT, PI_BAUD)
pub_arm_command = None
pub_gripper_command = None
home_pose = [0, 0, 0, 0, 0, 0]

# 命令队列和控制标志
command_queue = queue.Queue(maxsize=5)  # 限制队列大小防止积压
is_executing = False
last_command_time = 0
MIN_COMMAND_INTERVAL = 0.1  # 最小命令间隔(秒)

joint_names = [
    "joint2_to_joint1",
    "joint3_to_joint2", 
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

teleop_help = """\nTeleop Keyboard Controller (lowercase only)\n-------------------------------------------
w/s: joint1 ++/--
e/d: joint2 ++/--
r/f: joint3 ++/--
t/g: joint4 ++/--
y/h: joint5 ++/--
u/j: joint6 ++/--

o: open gripper
p: close gripper

1: init pose
q: quit
"""

# --------------------------- 串口自动检测函数 ---------------------------
def find_mycobot_port(baudrate=115200, timeout=3):
    """
    自动检测MyCobot机械臂的串口
    
    Args:
        baudrate: 波特率，默认115200
        timeout: 超时时间，默认3秒
    
    Returns:
        str: 找到的串口路径，如果未找到返回None
    """
    print("正在自动检测MyCobot机械臂串口...")
    
    # 获取所有可用串口
    available_ports = serial.tools.list_ports.comports()
    
    if not available_ports:
        print("未找到任何串口设备")
        return None
    
    print(f"找到 {len(available_ports)} 个串口设备：")
    for port in available_ports:
        print(f"  - {port.device}: {port.description}")
    
    # 优先检查常见的MyCobot串口设备
    priority_ports = []
    other_ports = []
    
    for port in available_ports:
        port_name = port.device.lower()
        description = port.description.lower() if port.description else ""
        
        # 检查是否是常见的MyCobot相关设备
        if any(keyword in description for keyword in ['ch340', 'ch341', 'cp210', 'ftdi', 'usb-serial']):
            priority_ports.append(port)
        elif 'ttyusb' in port_name or 'ttyacm' in port_name:
            priority_ports.append(port)
        else:
            other_ports.append(port)
    
    # 合并列表，优先检查可能的设备
    test_ports = priority_ports + other_ports
    
    # 测试每个串口
    for port in test_ports:
        print(f"正在测试串口: {port.device}")
        
        try:
            # 尝试创建MyCobot连接
            test_mc = MyCobot(port.device, baudrate)
            
            # 等待连接稳定
            time.sleep(1)
            
            # 尝试获取机械臂信息来验证连接
            try:
                # 尝试获取角度信息
                angles = test_mc.get_angles()
                if angles is not None and len(angles) == 6:
                    print(f"✓ 成功连接到MyCobot机械臂，串口: {port.device}")
                    print(f"  当前角度: {angles}")
                    return port.device
                else:
                    # 尝试其他验证方法
                    firmware_version = test_mc.get_system_version()
                    if firmware_version:
                        print(f"✓ 成功连接到MyCobot机械臂，串口: {port.device}")
                        print(f"  固件版本: {firmware_version}")
                        return port.device
            except Exception as e:
                print(f"  连接测试失败: {e}")
                
            # 清理测试连接
            try:
                test_mc.close()
            except:
                pass
                
        except Exception as e:
            print(f"  无法打开串口 {port.device}: {e}")
            continue
    
    print("❌ 未找到MyCobot机械臂设备")
    return None

def get_optimal_baudrate(port):
    """
    为指定串口找到最佳波特率
    
    Args:
        port: 串口路径
    
    Returns:
        int: 最佳波特率，如果未找到返回默认值115200
    """
    common_baudrates = [115200, 1000000, 500000, 256000, 230400, 57600, 38400, 19200, 9600]
    
    print(f"正在为串口 {port} 寻找最佳波特率...")
    
    for baud in common_baudrates:
        try:
            print(f"  测试波特率: {baud}")
            test_mc = MyCobot(port, baud)
            time.sleep(0.5)
            
            # 尝试通信测试
            try:
                angles = test_mc.get_angles()
                if angles is not None and len(angles) == 6:
                    print(f"✓ 最佳波特率: {baud}")
                    test_mc.close()
                    return baud
            except:
                pass
            
            test_mc.close()
            
        except Exception:
            continue
    
    print(f"使用默认波特率: 115200")
    return 115200

def smart_connect_mycobot():
    """
    智能连接MyCobot机械臂
    
    Returns:
        MyCobot: 连接成功的MyCobot对象，失败返回None
    """
    # 首先尝试ROS参数指定的串口
    port = rospy.get_param("~port", None)
    baud = rospy.get_param("~baud", 115200)
    
    if port:
        print(f"尝试使用ROS参数指定的串口: {port}")
        try:
            test_mc = MyCobot(port, baud)
            time.sleep(1)
            angles = test_mc.get_angles()
            if angles is not None and len(angles) == 6:
                print(f"✓ 成功连接到指定串口: {port}")
                return test_mc
            else:
                test_mc.close()
                print(f"指定串口 {port} 连接失败，开始自动检测")
        except Exception as e:
            print(f"指定串口 {port} 连接失败: {e}，开始自动检测")
    
    # 自动检测串口
    detected_port = find_mycobot_port()
    if not detected_port:
        return None
    
    # 找到最佳波特率
    optimal_baud = get_optimal_baudrate(detected_port)
    
    # 创建最终连接
    try:
        final_mc = MyCobot(detected_port, optimal_baud)
        time.sleep(1)
        print(f"✓ MyCobot机械臂连接成功!")
        print(f"  串口: {detected_port}")
        print(f"  波特率: {optimal_baud}")
        return final_mc
    except Exception as e:
        print(f"❌ 创建最终连接失败: {e}")
        return None

# --------------------------- 工具函数 ---------------------------
def deg_to_rad_list(deg_list):
    return [math.radians(a) for a in deg_list]

def map_gripper_value(value):
    gmin, gmax = 3, 91
    zmin, zmax = -0.68, 0.15
    return ((value - gmin) / (gmax - gmin)) * (zmax - zmin) + zmin

class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.prev = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.prev)

def publish_to_gazebo(angle_list):
    global pub_arm_command
    if pub_arm_command is None:
        pub_arm_command = rospy.Publisher(
            "/arm_controller/command", JointTrajectory, queue_size=10
        )

    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = deg_to_rad_list(angle_list)
    point.time_from_start = rospy.Duration(0.1)  # 减少执行时间
    traj.points = [point]

    pub_arm_command.publish(traj)

def publish_gripper_to_gazebo(gripper_value):
    global pub_gripper_command
    if pub_gripper_command is None:
        pub_gripper_command = rospy.Publisher(
            "/gripper_controller/command", JointTrajectory, queue_size=10
        )

    mapped = map_gripper_value(gripper_value)
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ["gripper_controller"]
    point = JointTrajectoryPoint()
    point.positions = [mapped]
    point.time_from_start = rospy.Duration(0.1)
    traj.points = [point]

    pub_gripper_command.publish(traj)

# --------------------------- 命令执行线程 ---------------------------
def command_executor():
    """异步执行命令的线程"""
    global is_executing, last_command_time
    
    while not rospy.is_shutdown():
        try:
            # 从队列获取命令，超时1秒
            command = command_queue.get(timeout=1.0)
            
            current_time = time.time()
            # 检查命令间隔，避免发送过于频繁
            if current_time - last_command_time < MIN_COMMAND_INTERVAL:
                time.sleep(MIN_COMMAND_INTERVAL - (current_time - last_command_time))
            
            is_executing = True
            
            if command['type'] == 'angles':
                angles = command['data']
                try:
                    # 增加执行速度，减少延迟
                    mc.send_angles(angles, 50)  # 提高速度从25到50
                    publish_to_gazebo(angles)
                    
                    # 发布ROS消息
                    pub_angles = rospy.Publisher(
                        "/joint_command_angles", Float64MultiArray, queue_size=1
                    )
                    pub_angles.publish(Float64MultiArray(data=angles))
                    
                except Exception as e:
                    rospy.logwarn(f"执行角度命令失败: {e}")
                    
            elif command['type'] == 'gripper':
                action = command['data']
                try:
                    mc.set_gripper_state(action, 80)  # 提高夹爪速度
                    gripper_value = mcs.get_gripper_value()
                    publish_gripper_to_gazebo(gripper_value)
                except Exception as e:
                    rospy.logwarn(f"执行夹爪命令失败: {e}")
            
            last_command_time = time.time()
            is_executing = False
            command_queue.task_done()
            
        except queue.Empty:
            # 队列为空，继续等待
            continue
        except Exception as e:
            rospy.logerr(f"命令执行器错误: {e}")
            is_executing = False

def add_command_to_queue(command_type, data):
    """添加命令到队列，如果队列满则静默丢弃最旧的命令"""
    command = {'type': command_type, 'data': data}
    
    try:
        # 非阻塞添加，如果队列满则丢弃
        command_queue.put_nowait(command)
    except queue.Full:
        # 队列满，静默丢弃最旧的命令，添加新命令
        try:
            command_queue.get_nowait()  # 移除最旧命令
            command_queue.put_nowait(command)  # 添加新命令
            # 移除了警告信息，静默处理
        except queue.Empty:
            pass

# --------------------------- 非阻塞键盘输入 ---------------------------
def get_key_non_blocking():
    """非阻塞获取按键"""
    import select
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

# --------------------------- 键盘控制 ---------------------------
def teleop_keyboard():
    global home_pose, mc

    angle_list = [0] * 6
    print(teleop_help)
    
    # 启动命令执行线程
    executor_thread = threading.Thread(target=command_executor, daemon=True)
    executor_thread.start()

    with RawTerminal():
        while not rospy.is_shutdown():
            key = get_key_non_blocking()
            
            if key is None:
                time.sleep(0.01)  # 短暂休眠减少CPU占用
                continue
                
            if key == 'q':
                break

            # 初始化回家
            if key == '1':
                angle_list = home_pose.copy()
                add_command_to_queue('angles', angle_list)
                print("回到初始位置")
                continue

            # 控制爪子开闭
            if key in ('o', 'p'):
                action = 0 if key == 'o' else 1
                add_command_to_queue('gripper', action)
                print(f"夹爪 {'打开' if action == 0 else '关闭'}")
                continue

            # 运动按键映射
            mapping = {
                'w': (0, +1), 's': (0, -1),
                'e': (1, +1), 'd': (1, -1),
                'r': (2, +1), 'f': (2, -1),
                't': (3, +1), 'g': (3, -1),
                'y': (4, +1), 'h': (4, -1),
                'u': (5, +1), 'j': (5, -1),
            }
            
            if key not in mapping:
                continue

            idx, step = mapping[key]
            
            # 简单的角度限制检查
            new_angle = angle_list[idx] + step
            if -180 <= new_angle <= 180:  # 基本角度限制
                angle_list[idx] = new_angle
                add_command_to_queue('angles', angle_list.copy())
                print(f"关节{idx+1}: {angle_list[idx]}°")
            else:
                print(f"关节{idx+1}超出限制范围")

# --------------------------- 主函数 ---------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("mycobot_keyboard_controller", anonymous=True)

        print("=" * 60)
        print("MyCobot机械臂键盘控制器 - 智能串口检测版")
        print("=" * 60)

        # 智能连接MyCobot机械臂
        mc = smart_connect_mycobot()
        
        if mc is None:
            print("❌ 无法连接到MyCobot机械臂，请检查：")
            print("  1. 机械臂是否正确连接到电脑")
            print("  2. 设备驱动是否正确安装")
            print("  3. 串口是否被其他程序占用")
            print("  4. 机械臂是否正常上电")
            sys.exit(1)

        # 释放所有舵机并稍作等待
        mc.release_all_servos()
        time.sleep(0.1)

        print("\n机械臂键盘控制器已启动")
        print("使用异步命令执行，减少延迟和卡顿")
        print("按 'q' 退出程序")
        
        teleop_keyboard()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        print("清理资源...")
        if mc:
            try:
                mc.release_all_servos()
                mc.close()
            except:
                pass
        print("程序已退出")
