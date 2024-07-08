#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import termios
import tty
import sys
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot.mycobot import MyCobot
import math

msg = """\
Mycobot_280_m5 Teleop Keyboard Controller
---------------------------
Movimg options (control the angle of each joint):
    w: joint2_to_joint1++   s: joint2_to_joint1--
    e: joint3_to_joint2++   d: joint3_to_joint2--
    r: joint4_to_joint3++   f: joint4_to_joint3--
    t: joint5_to_joint4++   g: joint5_to_joint4--
    y: joint6_to_joint5++   h: joint6_to_joint5--
    u: joint6output_to_joint6++ j: joint6output_to_joint6--

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
    # mc.set_fresh_mode(1)

    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    pub_command = rospy.Publisher("/mycobot_position_controller/command", JointTrajectory, queue_size=10)
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
        # Keyboard keys call different motion functions. 键盘按键调用不同的运动功能
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
                elif key in "2":
                    data_list = home_pose
                elif key in "3":
                    home_pose = data_list
                else:
                    continue

                traj_msg = JointTrajectory()
                traj_msg.header.stamp = rospy.Time.now()
                traj_msg.joint_names = joint_names

                radian_data_list = [angle * math.pi / 180 for angle in data_list]

                point = JointTrajectoryPoint()
                point.positions = radian_data_list
                point.velocities = [0.0] * len(joint_names)
                point.time_from_start = rospy.Duration(0.1)  # Set a small duration to continuously update

                traj_msg.points.append(point)
                pub_command.publish(traj_msg)

                mc.send_angles(data_list, 25)

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