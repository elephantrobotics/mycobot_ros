#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import termios
import tty
import sys
import rospy
import math
import time
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
    point.time_from_start = rospy.Duration(0.2)
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
    point.time_from_start = rospy.Duration(0.2)
    traj.points = [point]

    pub_gripper_command.publish(traj)

# --------------------------- 键盘控制 ---------------------------
def teleop_keyboard():
    global home_pose, mc

    pub_angles = rospy.Publisher(
        "/joint_command_angles", Float64MultiArray, queue_size=10
    )
    angle_list = [0] * 6
    print(teleop_help)

    with RawTerminal():
        while not rospy.is_shutdown():
            key = sys.stdin.read(1)
            if key == 'q':
                break

            # 初始化回家
            if key == '1':
                angle_list = home_pose.copy()
                pub_angles.publish(Float64MultiArray(data=angle_list))
                try:
                    mc.send_angles(angle_list, 25)
                    publish_to_gazebo(angle_list)
                except Exception:
                    print("已到超限位置，无法继续向前")
                continue

            # 控制爪子开闭
            if key in ('o', 'p'):
                action = 0 if key == 'o' else 1
                try:
                    mc.set_gripper_state(action, 70)
                    gripper_value = mcs.get_gripper_value()
                    publish_gripper_to_gazebo(gripper_value)
                except Exception:
                    print("已到超限位置，无法继续向前")
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
            angle_list[idx] += step
            pub_angles.publish(Float64MultiArray(data=angle_list))
            try:
                mc.send_angles(angle_list, 25)
                publish_to_gazebo(angle_list)
            except Exception:
                # 回滚上一步角度
                angle_list[idx] -= step
                print("已到超限位置，无法继续向前")
                continue

# --------------------------- 主函数 ---------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("mycobot_keyboard_controller", anonymous=True)

        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 115200)

        mc = MyCobot(port, baud)
        mc.release_all_servos()
        time.sleep(0.1)

        teleop_keyboard()

    except rospy.ROSInterruptException:
        pass
