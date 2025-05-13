#!/usr/bin/env python3
# encoding:utf-8
from pymycobot import MyArmM, MyArmC, MyCobot
from sensor_msgs.msg import JointState
import rospy
from std_msgs.msg import Header
from math import pi
import subprocess
import time
import copy
import threading
from queue import Queue
import os

# os.system('sudo chmod 777 /dev/ttyACM*')

class ArmController:
    def __init__(self):
        # ROS初始化
        rospy.init_node("multi_arm_controller", anonymous=True)
        self.shutdown_other_nodes()
        
        # 初始化机械臂连接
        self.myarm_c = MyArmC('/dev/ttyACM0', 1000000, debug=False)
        time.sleep(1)
        self.myarm_m = MyArmM('/dev/ttyACM1', 1000000, debug=False)
        time.sleep(1)
        self.mc_320 = MyCobot('/dev/ttyACM2', 115200, debug=False)
        time.sleep(1)
        self.mc_320.set_fresh_mode(1)
        self.mc_320.set_gripper_mode(0)
        # 使能MyArmM
        self.enable_myarm_m()
        
        # 创建线程间通信队列
        self.angle_queue = Queue(maxsize=1)
        self.angle2_queue = Queue(maxsize=1)
        self.stop_event = threading.Event()
        
        # ROS发布器
        self.pub_m = rospy.Publisher('myarm_m/joint_states', JointState, queue_size=10)
        self.pub_c = rospy.Publisher('myarm_c650/joint_states', JointState, queue_size=10)
        
        # 创建控制线程
        self.threads = [
            threading.Thread(target=self.angle_reader_thread, name="Angle-Reader"),
            threading.Thread(target=self.myarm_m_control_thread, name="MyArmM-Control"),
            threading.Thread(target=self.mycobot_320_control_thread, name="MyCobot320-Control"),
            threading.Thread(target=self.ros_publisher_thread, name="ROS-Publisher")
        ]
    
    def shutdown_other_nodes(self):
        """关闭其他可能冲突的ROS节点"""
        nodes = ['myarm_m/joint_state_publisher_gui', 'myarm_c650/joint_state_publisher_gui']
        for node in nodes:
            try:
                subprocess.run(['rosnode', 'kill', node], timeout=1)
            except:
                pass
    
    def enable_myarm_m(self):
        """使能MyArmM的所有关节"""
        for i in range(8):
            self.myarm_m.set_servo_enabled(i, 1)
            time.sleep(0.1)
    
    def linear_transform(self, x):
        """夹爪角度线性变换(MyArmC到仿真)"""
        return (0 - 0.022) / (0 - (-89.5)) * (x - (-89.5)) + 0.022
    
    def gripper_angle_transform(self, angle):
        """
        MyArmC夹爪角度到MyCobot320夹爪角度的转换
        MyArmC: 闭合=0, 打开=-118
        MyCobot320: 闭合=0, 打开=100
        """
        # 将MyArmC的角度范围[0, -118]映射到MyCobot320的[0, 100]
        # 首先将MyArmC角度转换为0-118的正值范围
        normalized_angle = abs(angle)
        # 然后映射到0-100范围
        return int((normalized_angle / 118) * 100)
    
    def get_current_angles(self):
        """获取当前MyArmC的角度并放入队列"""
        while not self.stop_event.is_set():
            angles = self.myarm_c.get_joints_angle()
            if self.angle_queue.empty():
                if angles == None:
                    continue
                self.angle_queue.put(angles)
                a2 = angles.copy()
                # print("angles get " ,angles)
                self.angle2_queue.put(a2)
            time.sleep(0.01)  # 10ms更新频率
    
    def angle_reader_thread(self):
        """角度读取线程"""
        rospy.loginfo("Angle reader thread started")
        self.get_current_angles()
    
    def myarm_m_control_thread(self):
        """MyArmM控制线程"""
        rospy.loginfo("MyArmM control thread started")
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        t2 = time.time()
        while not self.stop_event.is_set():
            if not self.angle_queue.empty():
                angles = self.angle_queue.get()
                # angles = [0,0,0,0,0,0,0]
                
                # 角度转换
                anglesm = copy.deepcopy(angles)
                gripper_angle = anglesm.pop(6)
                angle_m = [a*pi/180 for a in anglesm]
                
                # 夹爪处理
                gripper_sim = self.linear_transform(gripper_angle)/0.022*0.0345
                angle_m.append(gripper_sim)
                angle_m[1] *= -1  # 关节2方向反转
                
                # 发布ROS消息
                joint_state.header = Header(stamp=rospy.Time.now())
                joint_state.position = angle_m
                self.pub_m.publish(joint_state)
                
                # 设置实际角度
                gripper_real = gripper_sim * (-3500)
                angle_m = [a*180/pi for a in angle_m[:6]] + [max(-117.5, min(-0.1, gripper_real))]
                self.myarm_m.set_joints_angle(angle_m, 30)
                # print('time:', 1000*(t2-time.time()))
                t2 = time.time()
    
    def mycobot_320_control_thread(self):
        """MyCobot320控制线程"""
        rospy.loginfo("MyCobot320 control thread started")
        self.mc_320.set_fresh_mode(1)
        con = 0
        t2 = time.time()
        while not self.stop_event.is_set():
            if not self.angle2_queue.empty():
                angles = copy.deepcopy(self.angle2_queue.queue[0])  # 获取最新角度不取出
                
                # 角度转换 - 根据实际机械臂特性调整这些参数
                angles_320 = [
                    angles[0], 
                    angles[1],
                    -angles[2]-90,
                    -angles[4] +90,         
                    angles[3] + 80,        
                    angles[5]     
                ]
                if con >= 0:
                    con = 0
                else:
                    con += 1
                    continue

                # 发送控制命令
                self.mc_320.send_angles(angles_320, 100)
                
                print(angles_320)
                print('time:', 1000*(t2-time.time()))
                t2 = time.time()
                
                # 控制MyCobot320的夹爪
                gripper_angle = angles[6]  # 获取MyArmC的夹爪角度
                mc320_gripper_value = self.gripper_angle_transform(gripper_angle)
                # self.mc_320.set_gripper_value(mc320_gripper_value, 50)  # 速度设为50
                
                time.sleep(0.01)  # 控制频率约100Hz
    
    def ros_publisher_thread(self):
        """ROS发布线程(MyArmC状态)"""
        rospy.loginfo("ROS publisher thread started")
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        rate = rospy.Rate(100)  # 100Hz
        
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            if not self.angle_queue.empty():
                angles = copy.deepcopy(self.angle_queue.queue[0])
                
                # 角度转换
                gripper_angle = angles.pop(6)
                angle_c = [a/180*pi for a in angles]
                angle_c.append(self.linear_transform(gripper_angle))
                
                # 发布ROS消息
                joint_state.header = Header(stamp=rospy.Time.now())
                joint_state.position = angle_c
                self.pub_c.publish(joint_state)
            
            rate.sleep()
    
    def run(self):
        """启动所有线程"""
        for thread in self.threads:
            thread.daemon = True
            thread.start()
        
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("All control threads started")
        
        try:
            while not rospy.is_shutdown():
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.shutdown()
    
    def shutdown(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=1)
        rospy.loginfo("All threads stopped")
        rospy.signal_shutdown("Controller shutdown")

if __name__ == '__main__':
    controller = ArmController()
    controller.run()