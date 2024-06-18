#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyUSB0'
    baud: serial prot baudrate. Defaults is 115200.
"""
import time
import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot.mycobot import MyCobot
import copy

mc, pub_command = None, None

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)

    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = rospy.Time.now()
    joint_trajectory.joint_names = ["joint2_to_joint1","joint3_to_joint2","joint4_to_joint3","joint5_to_joint4","joint6_to_joint5","joint6output_to_joint6"]

    point = JointTrajectoryPoint()
    point.positions = data.position
    point.velocities = [0.0] * len(joint_trajectory.joint_names)
    point.time_from_start = rospy.Duration(0.5)
    joint_trajectory.points.append(point)

    pub_command.publish(joint_trajectory)  # Publish the joint trajectory
    mc.send_angles(data_list, 25)
    # rospy.loginfo(rospy.get_caller_id() + "%s", data_list)

def listener():
    global mc, pub_command

    rospy.init_node("control_slider", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)

    pub_command = rospy.Publisher("/mycobot_position_controller/command", JointTrajectory, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, callback)

    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)

    # spin() simply keeps python from exiting until this node is stopped
    # spin()只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()

if __name__ == "__main__":
    listener()