#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import termios
import tty
import sys
import rospy
import math
import time
import threading
from pymycobot import PI_PORT, PI_BAUD 
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot import MyCobot280

# --------------------------- 全局变量 ---------------------------
mc = None
pub_arm_command = None
pub_gripper_command = None
home_pose = [0, 0, 0, 0, 0, 0]

joint_names = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

teleop_help = """\
Mycobot_280_m5 Teleop Keyboard Controller
---------------------------
Moving options (control each joint):
    w/s: joint1 ++/--
    e/d: joint2 ++/--
    r/f: joint3 ++/--
    t/g: joint4 ++/--
    y/h: joint5 ++/--
    u/j: joint6 ++/--

Gripper control:
    o: Open gripper
    p: Close gripper

Other:
    1 - Go to init pose
    2 - Go to home pose
    3 - Save current as home pose
    q - Quit
"""

# --------------------------- 工具类 ---------------------------
class Raw:
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)

def deg_to_rad_list(deg_list):
    return [math.radians(angle) for angle in deg_list]

def map_gripper_value(value):
    mycobot_min, mycobot_max = 3, 91
    gazebo_min, gazebo_max = -0.68, 0.15
    return ((value - mycobot_min) / (mycobot_max - mycobot_min)) * (gazebo_max - gazebo_min) + gazebo_min

# --------------------------- 键盘控制主函数 ---------------------------
def teleop_keyboard():
    global home_pose

    pub_angles = rospy.Publisher("/joint_command_angles", Float64MultiArray, queue_size=10)
    rate = rospy.Rate(30)
    angle_list = [0, 0, 0, 0, 0, 0]

    print(teleop_help)

    while not rospy.is_shutdown():
        try:
            with Raw(sys.stdin):
                key = sys.stdin.read(1)

            if key == "q":
                break

            # 关节控制
            if key.lower() in "wsedrftgyhuj":
                idx = "wsedrftgyhuj".index(key.lower()) // 2
                step = 1 if key.islower() else -1
                angle_list[idx] += step
            elif key == "1":
                angle_list = [0, 0, 0, 0, 0, 0]
            elif key == "2":
                angle_list = list(home_pose)
            elif key == "3":
                home_pose = list(angle_list)
            elif key == "o":
                mc.set_gripper_state(0, 70)
            elif key == "p":
                mc.set_gripper_state(1, 70)
            else:
                continue

            angle_msg = Float64MultiArray(data=angle_list)
            pub_angles.publish(angle_msg)
            mc.send_angles(angle_list, 25)

        except Exception as e:
            rospy.logerr(f"Teleop Error: {e}")

# --------------------------- 控制回调 ---------------------------
def command_callback(msg):
    global pub_arm_command

    joint_angles = msg.data
    if len(joint_angles) != 6:
        rospy.logwarn("Expected 6 joint angles.")
        return

    rad_angles = deg_to_rad_list(joint_angles)
    point = JointTrajectoryPoint(
        positions=rad_angles,
        velocities=[0.0] * 6,
        time_from_start=rospy.Duration(0.1)
    )

    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = joint_names
    traj.points.append(point)

    pub_arm_command.publish(traj)

def callback(data):
    global mc, pub_arm_command, pub_gripper_command

    try:
        angles = mc.get_angles()
        if not isinstance(angles, list) or len(angles) != 6:
            rospy.logwarn(f"Invalid joint angles received: {angles}")
            return

        gripper_value = mc.get_gripper_value()
        if not isinstance(gripper_value, (int, float)):
            rospy.logwarn(f"Invalid gripper value received: {gripper_value}")
            return

        rad_angles = deg_to_rad_list(angles)

        # Arm trajectory
        arm_point = JointTrajectoryPoint(
            positions=rad_angles,
            velocities=[0.0] * 6,
            time_from_start=rospy.Duration(0.5)
        )
        arm_traj = JointTrajectory(
            header=rospy.Header(stamp=rospy.Time.now()),
            joint_names=joint_names,
            points=[arm_point]
        )
        pub_arm_command.publish(arm_traj)

        # Gripper trajectory
        mapped_gripper = map_gripper_value(gripper_value)
        gripper_point = JointTrajectoryPoint(
            positions=[mapped_gripper],
            velocities=[0.0],
            time_from_start=rospy.Duration(0.5)
        )
        gripper_traj = JointTrajectory(
            header=rospy.Header(stamp=rospy.Time.now()),
            joint_names=["gripper_controller"],
            points=[gripper_point]
        )
        pub_gripper_command.publish(gripper_traj)

    except Exception as e:
        rospy.logerr(f"Callback error: {e}")

# --------------------------- ROS 初始化 ---------------------------
def listener():
    global mc, pub_arm_command, pub_gripper_command

    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)

    try:
        mc = MyCobot280(port, baud)
        mc.release_all_servos()
        rospy.loginfo("Connected to MyCobot.")
    except Exception as e:
        rospy.logerr(f"Failed to connect to MyCobot: {e}")
        return

    pub_arm_command = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
    pub_gripper_command = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)

    rospy.Subscriber("/joint_command_angles", Float64MultiArray, command_callback)
    rospy.Subscriber("/joint_states", JointState, callback)

    rospy.spin()

# --------------------------- 主程序入口 ---------------------------
# --------------------------- 主程序入口 ---------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("mycobot_keyboard_controller", anonymous=True)

        # 启动监听线程
        t1 = threading.Thread(target=listener)
        t1.daemon = True
        t1.start()

        # 启动键盘控制
        teleop_keyboard()

    except rospy.ROSInterruptException:
        pass
