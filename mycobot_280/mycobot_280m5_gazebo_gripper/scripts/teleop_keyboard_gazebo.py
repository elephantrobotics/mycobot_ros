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
mc280 = None  # 专门用于获取夹爪值的MyCobot280实例
pub_arm_command = None
pub_gripper_command = None
pub_angles = None  # 添加全局角度发布器
home_pose = [0, 0, 0, 0, 0, 0]

# 存储连接信息
connected_port = None
connected_baud = None

# 命令队列和控制标志
command_queue = queue.Queue(maxsize=3)  # 减少队列大小，减少延迟
is_executing = False
last_command_time = 0
last_gripper_command_time = 0
MIN_COMMAND_INTERVAL = 0.05  # 减少命令间隔，提高响应速度
MIN_GRIPPER_INTERVAL = 0.5   # 夹爪命令间隔，避免频繁切换
stop_executor = False
current_gripper_state = None  # 记录当前夹爪状态

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

o: open gripper  (现实打开，Gazebo打开)
p: close gripper (现实关闭，Gazebo关闭)

1: init pose
q: quit

注意：现实夹爪 0=打开,1=关闭; MyCobot值 3=打开,91=关闭
"""

# --------------------------- 串口自动检测函数 ---------------------------
def find_mycobot_port(baudrate=115200, timeout=3):
    """
    自动检测MyCobot机械臂的串口，优先考虑 /dev/ttyUSB* 和 /dev/ttyACM*
    """
    print("正在自动检测MyCobot机械臂串口...")

    available_ports = serial.tools.list_ports.comports()
    if not available_ports:
        print("未找到任何串口设备")
        return None, None

    filtered_ports = []
    for port in available_ports:
        port_name = port.device.lower()
        desc = port.description.lower() if port.description else ""
        if "ama0" in port_name:
            continue  # 跳过默认的 AMA0
        if "usb" in port_name or "acm" in port_name or any(x in desc for x in ['ch340', 'ch341', 'cp210', 'ftdi', 'serial']):
            filtered_ports.append(port)

    print(f"发现 {len(filtered_ports)} 个可能的 MyCobot 设备：")
    for port in filtered_ports:
        print(f"  - {port.device}: {port.description}")

    for port in filtered_ports:
        print(f"正在测试串口: {port.device}")
        optimal_baud = get_optimal_baudrate(port.device)
        try:
            test_mc = MyCobot(port.device, optimal_baud)
            time.sleep(1)
            angles = test_mc.get_angles()
            if angles and len(angles) == 6:
                print(f"✓ 成功连接到MyCobot设备: {port.device}, 波特率: {optimal_baud}, 角度: {angles}")
                test_mc.close()
                return port.device, optimal_baud
        except Exception as e:
            print(f"  测试失败: {e}")
        finally:
            try:
                test_mc.close()
            except:
                pass

    print("❌ 未能连接任何USB串口上的MyCobot设备")
    return None, None

def get_optimal_baudrate(port):
    """
    为指定串口找到最佳波特率
    """
    common_baudrates = [115200, 1000000, 500000, 256000, 230400, 57600, 38400, 19200, 9600]
    
    print(f"  正在为串口 {port} 寻找最佳波特率...")
    
    for baud in common_baudrates:
        try:
            print(f"    测试波特率: {baud}")
            test_mc = MyCobot(port, baud)
            time.sleep(0.5)
            
            # 尝试通信测试
            try:
                angles = test_mc.get_angles()
                if angles is not None and len(angles) == 6:
                    print(f"    ✓ 找到工作波特率: {baud}")
                    test_mc.close()
                    return baud
            except:
                pass
            
            test_mc.close()
            
        except Exception:
            continue
    
    print(f"    使用默认波特率: 115200")
    return 115200

def smart_connect_mycobot():
    """
    智能连接MyCobot机械臂，返回连接的实例和连接信息
    """
    global connected_port, connected_baud, mc280
    
    port = rospy.get_param("~port", None)
    baud = rospy.get_param("~baud", 115200)

    if port and "ama0" not in port.lower():
        print(f"尝试使用ROS指定串口: {port}")
        try:
            test_mc = MyCobot(port, baud)
            time.sleep(1)
            angles = test_mc.get_angles()
            if angles and len(angles) == 6:
                print(f"✓ 成功连接到指定串口: {port}")
                connected_port = port
                connected_baud = baud
                # 创建MyCobot280实例用于夹爪控制
                try:
                    mc280 = MyCobot280(port, baud)
                    print("✓ MyCobot280实例创建成功，可获取夹爪值")
                except Exception as e:
                    print(f"MyCobot280实例创建失败: {e}")
                    mc280 = None
                return test_mc
            else:
                test_mc.close()
        except Exception as e:
            print(f"  指定串口连接失败: {e}")

    detected_port, optimal_baud = find_mycobot_port()
    if not detected_port:
        return None

    try:
        final_mc = MyCobot(detected_port, optimal_baud)
        time.sleep(1)
        connected_port = detected_port
        connected_baud = optimal_baud
        print(f"✓ 成功连接MyCobot: {detected_port} @ {optimal_baud} baud")
        
        # 创建MyCobot280实例用于夹爪控制
        try:
            mc280 = MyCobot280(detected_port, optimal_baud)
            print("✓ MyCobot280实例创建成功，可获取夹爪值")
        except Exception as e:
            print(f"MyCobot280实例创建失败: {e}")
            mc280 = None
            
        return final_mc
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return None

# --------------------------- 工具函数 ---------------------------
def deg_to_rad_list(deg_list):
    return [math.radians(a) for a in deg_list]

def map_gripper_value(value):
    """
    映射夹爪值到Gazebo控制器
    value: 夹爪值 (3=打开, 91=关闭)
    返回: Gazebo控制器值 (-0.68到0.15)
    """
    gmin, gmax = 3, 91  # MyCobot夹爪值范围：3=打开，91=关闭
    zmin, zmax = 0.15, -0.68  # 反转映射：打开对应0.15，关闭对应-0.68
    return ((value - gmin) / (gmax - gmin)) * (zmax - zmin) + zmin

class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.prev = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.prev)

def initialize_publishers():
    """初始化所有ROS发布器"""
    global pub_arm_command, pub_gripper_command, pub_angles
    
    pub_arm_command = rospy.Publisher(
        "/arm_controller/command", JointTrajectory, queue_size=10
    )
    
    pub_gripper_command = rospy.Publisher(
        "/gripper_controller/command", JointTrajectory, queue_size=10
    )
    
    pub_angles = rospy.Publisher(
        "/joint_command_angles", Float64MultiArray, queue_size=1
    )
    
    print("ROS发布器初始化完成")

def publish_to_gazebo(angle_list):
    global pub_arm_command
    
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = deg_to_rad_list(angle_list)
    point.time_from_start = rospy.Duration(0.1)
    traj.points = [point]

    pub_arm_command.publish(traj)

def publish_gripper_to_gazebo(gripper_value):
    global pub_gripper_command
    
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
    global is_executing, last_command_time, last_gripper_command_time, stop_executor, current_gripper_state
    
    while not rospy.is_shutdown() and not stop_executor:
        try:
            # 从队列获取命令，超时0.5秒减少等待时间
            command = command_queue.get(timeout=0.5)
            
            current_time = time.time()
            
            if command['type'] == 'angles':
                # 角度命令的频率控制
                if current_time - last_command_time < MIN_COMMAND_INTERVAL:
                    time.sleep(MIN_COMMAND_INTERVAL - (current_time - last_command_time))
                
                is_executing = True
                angles = command['data']
                try:
                    # 提高机械臂速度，减少执行时间
                    mc.send_angles(angles, 80)  # 从50提高到80
                    publish_to_gazebo(angles)
                    
                    # 发布ROS消息
                    pub_angles.publish(Float64MultiArray(data=angles))
                    
                except Exception as e:
                    rospy.logwarn(f"执行角度命令失败: {e}")
                
                last_command_time = time.time()
                    
            elif command['type'] == 'gripper':
                # 夹爪命令的频率控制
                if current_time - last_gripper_command_time < MIN_GRIPPER_INTERVAL:
                    # 跳过过于频繁的夹爪命令
                    command_queue.task_done()
                    continue
                
                is_executing = True
                action = command['data']
                
                # 检查是否需要执行（状态是否改变）
                if current_gripper_state == action:
                    print(f"夹爪已经是{'关闭' if action == 1 else '打开'}状态，跳过")
                    command_queue.task_done()
                    is_executing = False
                    continue
                
                try:
                    # 执行夹爪命令
                    mc.set_gripper_state(action, 100)  # 提高夹爪速度
                    current_gripper_state = action  # 更新状态记录
                    
                    # 等待夹爪动作完成
                    time.sleep(0.3)
                    
                    # 尝试获取夹爪值
                    gripper_value = None
                    
                    # 首先尝试使用mc280实例获取夹爪值
                    if mc280 is not None:
                        try:
                            gripper_value = mc280.get_gripper_value()
                            if gripper_value is not None:
                                print(f"实际夹爪值: {gripper_value}")
                        except Exception as e:
                            rospy.logdebug(f"mc280获取夹爪值失败: {e}")
                    
                    # 如果mc280获取失败，尝试用mc获取
                    if gripper_value is None:
                        try:
                            if hasattr(mc, 'get_gripper_value'):
                                gripper_value = mc.get_gripper_value()
                                if gripper_value is not None:
                                    print(f"实际夹爪值: {gripper_value}")
                        except Exception as e:
                            rospy.logdebug(f"mc获取夹爪值失败: {e}")
                    
                    # 如果都获取不到，使用基于状态的预测值
                    if gripper_value is None:
                        gripper_value = 3 if action == 0 else 91  # 0=打开=3, 1=关闭=91
                        print(f"使用预测夹爪值: {gripper_value} ({'打开' if action == 0 else '关闭'})")
                    
                    # 立即发布到Gazebo
                    publish_gripper_to_gazebo(gripper_value)
                    print(f"夹爪 {'打开' if action == 0 else '关闭'} 完成，Gazebo值: {map_gripper_value(gripper_value):.3f}")
                        
                except Exception as e:
                    rospy.logwarn(f"执行夹爪命令失败: {e}")
                    # 即使出错也发布默认值到Gazebo
                    default_value = 3 if action == 0 else 91  # 0=打开=3, 1=关闭=91
                    publish_gripper_to_gazebo(default_value)
                    current_gripper_state = action  # 仍然更新状态
                    print(f"夹爪命令出错，使用默认值: {default_value}, Gazebo值: {map_gripper_value(default_value):.3f}")
                
                last_gripper_command_time = time.time()
            
            is_executing = False
            command_queue.task_done()
            
        except queue.Empty:
            # 队列为空，继续等待
            continue
        except Exception as e:
            rospy.logerr(f"命令执行器错误: {e}")
            is_executing = False

def add_command_to_queue(command_type, data):
    """添加命令到队列，智能处理不同类型命令"""
    command = {'type': command_type, 'data': data}
    
    if command_type == 'gripper':
        # 夹爪命令：清空队列中的其他夹爪命令，确保最新命令被执行
        temp_commands = []
        while not command_queue.empty():
            try:
                old_command = command_queue.get_nowait()
                if old_command['type'] != 'gripper':
                    temp_commands.append(old_command)
            except queue.Empty:
                break
        
        # 重新加入非夹爪命令
        for cmd in temp_commands:
            try:
                command_queue.put_nowait(cmd)
            except queue.Full:
                break
        
        # 添加新的夹爪命令
        try:
            command_queue.put_nowait(command)
            print(f"夹爪命令已加入队列: {'关闭' if data == 1 else '打开'}")
        except queue.Full:
            print("夹爪命令队列满，忽略")
    
    else:
        # 角度命令：正常队列处理
        try:
            command_queue.put_nowait(command)
        except queue.Full:
            # 队列满，移除最旧的角度命令
            try:
                old_command = command_queue.get_nowait()
                if old_command['type'] == 'angles':  # 只移除角度命令
                    command_queue.put_nowait(command)
            except queue.Empty:
                pass

# --------------------------- 非阻塞键盘输入 ---------------------------
def get_key_non_blocking():
    """非阻塞获取按键"""
    import select
    try:
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
    except Exception:
        pass
    return None

# --------------------------- 键盘控制 ---------------------------
def teleop_keyboard():
    global home_pose, mc, stop_executor, current_gripper_state

    angle_list = [0] * 6
    print(teleop_help)
    
    # 启动命令执行线程
    executor_thread = threading.Thread(target=command_executor, daemon=True)
    executor_thread.start()

    with RawTerminal():
        while not rospy.is_shutdown():
            key = get_key_non_blocking()
            
            if key is None:
                time.sleep(0.005)  # 减少休眠时间，提高响应速度
                continue
                
            if key == 'q':
                stop_executor = True
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
                
                # 检查是否重复按键
                if current_gripper_state == action:
                    print(f"夹爪已经是{'关闭' if action == 1 else '打开'}状态")
                    continue
                
                add_command_to_queue('gripper', action)
                print(f"发送夹爪命令: {'关闭' if action == 1 else '打开'}")
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
            if -180 <= new_angle <= 180:
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

        print(f"已连接串口: {connected_port} @ {connected_baud} baud")
        
        # 初始化ROS发布器
        initialize_publishers()
        
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
        stop_executor = True
        if mc:
            try:
                mc.release_all_servos()
                mc.close()
            except Exception as e:
                print(f"清理mc资源时出错: {e}")
        if mc280:
            try:
                mc280.close()
            except Exception as e:
                print(f"清理mc280资源时出错: {e}")
        print("程序已退出")
