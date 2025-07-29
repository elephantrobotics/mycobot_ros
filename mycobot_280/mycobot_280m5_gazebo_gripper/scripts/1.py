#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import termios
import tty
import sys
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
from pymycobot import MyCobot280 
import math

msg = """\
Mycobot_280_m5 Teleop Keyboard Controller
---------------------------
Moving options (control the angle of each joint):
    w: joint2_to_joint1++   s: joint2_to_joint1--
    e: joint3_to_joint2++   d: joint3_to_joint2--
    r: joint4_to_joint3++   f: joint4_to_joint3--
    t: joint5_to_joint4++   g: joint5_to_joint4--
    y: joint6_to_joint5++   h: joint6_to_joint5--
    u: joint6output_to_joint6++ j: joint6output_to_joint6--

Gripper control:
    x: Open gripper
    z: Close gripper

Other:
    1 - Go to init pose
    2 - Go to home pose
    3 - Resave home pose
    q - Quit
"""

home_pose = []
joint_names = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

gripper_state = 0  # 0 - open, 1 - closed
gripper_names = ["gripper_controller"]

class Raw(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)

def teleop_keyboard():
    rospy.init_node("teleop_keyboard")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    mc = MyCobot(port, baud)

    # Gazebo arm and gripper control publishers
    pub_arm_command = rospy.Publisher("/mycobot_position_controller/command", JointTrajectory, queue_size=10)
    pub_gripper_command = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
    
    rate = rospy.Rate(30)

    joint_state_send = JointState()
    joint_state_send.header = Header()
    joint_state_send.name = joint_names
    joint_state_send.velocity = [0] * len(joint_state_send.name)
    joint_state_send.effort = []

    data_list = [0, 0, 0, 0, 0, 0]
    mc.send_angles(data_list, 25)

    try:
        print(msg)
        # Keyboard keys call different motion functions.
        while True:
            try:
                with Raw(sys.stdin):
                    key = sys.stdin.read(1)
                if key == "q":
                    break
                elif key in ["w", "W"]:
                    if data_list[0] < 168: data_list[0] += 1
                elif key in ["s", "S"]:
                    if data_list[0] > -168: data_list[0] -= 1
                elif key in ["e", "E"]:
                    if data_list[1] < 135:  data_list[1] += 1
                elif key in ["d", "D"]:
                    if data_list[1] > -135: data_list[1] -= 1
                elif key in ["r", "R"]:
                    if data_list[2] < 150:  data_list[2] += 1
                elif key in ["f", "F"]:
                    if data_list[2] > -150: data_list[2] -= 1
                elif key in ["t", "T"]:
                    if data_list[3] < 145:  data_list[3] += 1
                elif key in ["g", "G"]:
                    if data_list[3] > -145: data_list[3] -= 1
                elif key in ["y", "Y"]:
                    if data_list[4] < 165:  data_list[4] += 1
                elif key in ["h", "H"]:
                    if data_list[4] > -165: data_list[4] -= 1
                elif key in ["u", "U"]:
                    if data_list[5] < 180:  data_list[5] += 1
                elif key in ["j", "J"]:
                    if data_list[5] > -180: data_list[5] -= 1
                elif key == "1":
                    data_list = [0, 0, 0, 0, 0, 0]
                elif key == "2":
                    data_list = home_pose
                elif key == "3":
                    home_pose = data_list
                elif key == "x":  # Open gripper
                    gripper_state = 0
                elif key == "z":  # Close gripper
                    gripper_state = 1
                else:
                    continue

                # Send arm commands to Gazebo (for visualization)
                traj_msg = JointTrajectory()
                traj_msg.header.stamp = rospy.Time.now()
                traj_msg.joint_names = joint_names

                radian_data_list = [angle * math.pi / 180 for angle in data_list]

                point = JointTrajectoryPoint()
                point.positions = radian_data_list
                point.velocities = [0.0] * len(joint_names)
                point.time_from_start = rospy.Duration(0.1)  # Set a small duration to continuously update
                traj_msg.points.append(point)
                pub_arm_command.publish(traj_msg)

                # Send gripper command to Gazebo
                gripper_position = 0.0 if gripper_state == 0 else 0.68  # Mapping from open (0) to close (0.68)
                gripper_trajectory = JointTrajectory()
                gripper_trajectory.header.stamp = rospy.Time.now()
                gripper_trajectory.joint_names = gripper_names
                gripper_point = JointTrajectoryPoint()
                gripper_point.positions = [gripper_position]
                gripper_point.velocities = [0.0]  # No velocity for gripper
                gripper_point.time_from_start = rospy.Duration(0.1)
                gripper_trajectory.points.append(gripper_point)
                pub_gripper_command.publish(gripper_trajectory)

                # Send real arm movement commands
                mc.send_angles(data_list, 25)

                # Control the real gripper
                mc.set_gripper_value(gripper_state)

            except Exception as e:
                continue

    except Exception as e:
        print(e)

    rate.sleep()

if __name__ == "__main__":
    try:
        teleop_keyboard()
    except rospy.ROSInterruptException:
        pass