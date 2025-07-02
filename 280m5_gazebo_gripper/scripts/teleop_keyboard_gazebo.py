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
from sensor_msgs.msg import JointState
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

teleop_help = """\nTeleop Keyboard Controller (lowercase only)\n-------------------------------------------\nw/s: joint1 ++/--\ne/d: joint2 ++/--\nr/f: joint3 ++/--\nt/g: joint4 ++/--\ny/h: joint5 ++/--\nu/j: joint6 ++/--\n\no: open gripper\np: close gripper\n\n1: init pose\nq: quit\n"""

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
        pub_arm_command = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)

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
        pub_gripper_command = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)

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

    pub_angles = rospy.Publisher("/joint_command_angles", Float64MultiArray, queue_size=10)
    angle_list = [0] * 6
    print(teleop_help)

    with RawTerminal():
        while not rospy.is_shutdown():
            key = sys.stdin.read(1)
            if key == 'q':
                break

            if   key == 'w': idx, step = 0, +1
            elif key == 's': idx, step = 0, -1
            elif key == 'e': idx, step = 1, +1
            elif key == 'd': idx, step = 1, -1
            elif key == 'r': idx, step = 2, +1
            elif key == 'f': idx, step = 2, -1
            elif key == 't': idx, step = 3, +1
            elif key == 'g': idx, step = 3, -1
            elif key == 'y': idx, step = 4, +1
            elif key == 'h': idx, step = 4, -1
            elif key == 'u': idx, step = 5, +1
            elif key == 'j': idx, step = 5, -1
            elif key == '1':
                angle_list = home_pose.copy()
                pub_angles.publish(Float64MultiArray(data=angle_list))
                mc.send_angles(angle_list, 25)
                publish_to_gazebo(angle_list)
                continue
            elif key == 'o':
                mc.set_gripper_state(0, 70)
                gripper_value = mcs.get_gripper_value()
                publish_gripper_to_gazebo(gripper_value)
                continue
            elif key == 'p':
                mc.set_gripper_state(1, 70)
                gripper_value = mcs.get_gripper_value()
                publish_gripper_to_gazebo(gripper_value)
                continue
            else:
                continue

            angle_list[idx] += step
            pub_angles.publish(Float64MultiArray(data=angle_list))
            mc.send_angles(angle_list, 25)
            publish_to_gazebo(angle_list)

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
