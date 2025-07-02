#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial port string. Defaults is '/dev/ttyUSB0'
    baud: serial port baudrate. Defaults is 115200.
"""
import time
import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

mc, pub_arm_command, pub_gripper_command = None, None, None

def callback(data):
    # Define the expected joint names for arm and gripper
    expected_arm_joint_names = [
        "joint2_to_joint1", 
        "joint3_to_joint2", 
        "joint4_to_joint3", 
        "joint5_to_joint4", 
        "joint6_to_joint5", 
        "joint6output_to_joint6"
    ]
    
    expected_gripper_joint_names = [
        "gripper_controller"
    ]

    # Filter the received joint states based on the expected joint names
    filtered_arm_positions = []
    filtered_gripper_positions = []

    for name, position in zip(data.name, data.position):
        if name in expected_arm_joint_names:
            radians_to_angles = round(math.degrees(position), 2)
            filtered_arm_positions.append(radians_to_angles)
        elif name in expected_gripper_joint_names:
            radians_to_angles = round(math.degrees(position), 2)
            filtered_gripper_positions.append(radians_to_angles)

    # Ensure the number of joints matches
    expected_num_arm_joints = len(expected_arm_joint_names)
    expected_num_gripper_joints = len(expected_gripper_joint_names)

    if len(filtered_arm_positions) != expected_num_arm_joints:
        rospy.logerr(f"Received {len(filtered_arm_positions)} arm positions, but expected {expected_num_arm_joints} positions.")
        return

    if len(filtered_gripper_positions) != expected_num_gripper_joints:
        rospy.logerr(f"Received {len(filtered_gripper_positions)} gripper positions, but expected {expected_num_gripper_joints} positions.")
        return

    # Create and publish arm joint trajectory
    arm_joint_trajectory = JointTrajectory()
    arm_joint_trajectory.header.stamp = rospy.Time.now()
    arm_joint_trajectory.joint_names = expected_arm_joint_names

    arm_point = JointTrajectoryPoint()
    arm_point.positions = [math.radians(pos) for pos in filtered_arm_positions]  # Convert back to radians for the command
    arm_point.velocities = [0.0] * expected_num_arm_joints  # Initialize velocities to zero
    arm_point.accelerations = [0.0] * expected_num_arm_joints  # Initialize accelerations to zero
    arm_point.time_from_start = rospy.Duration(0.5)
    arm_joint_trajectory.points.append(arm_point)

    pub_arm_command.publish(arm_joint_trajectory)  # Publish the arm joint trajectory

    # Create and publish gripper joint trajectory
    gripper_joint_trajectory = JointTrajectory()
    gripper_joint_trajectory.header.stamp = rospy.Time.now()
    gripper_joint_trajectory.joint_names = expected_gripper_joint_names

    gripper_point = JointTrajectoryPoint()
    gripper_point.positions = [math.radians(pos) for pos in filtered_gripper_positions]  # Convert back to radians for the command
    gripper_point.velocities = [0.0] * expected_num_gripper_joints  # Initialize velocities to zero
    gripper_point.accelerations = [0.0] * expected_num_gripper_joints  # Initialize accelerations to zero
    gripper_point.time_from_start = rospy.Duration(0.5)
    gripper_joint_trajectory.points.append(gripper_point)

    pub_gripper_command.publish(gripper_joint_trajectory)  # Publish the gripper joint trajectory

def listener():
    global mc, pub_arm_command, pub_gripper_command

    rospy.init_node("control_slider", anonymous=True)

    # port = rospy.get_param("~port", "/dev/ttyUSB0")
    # baud = rospy.get_param("~baud", 115200)
    # print(port, baud)
    # mc = MyCobot(port, baud)

    pub_arm_command = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
    pub_gripper_command = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, callback)

    # time.sleep(0.05)
    # mc.set_fresh_mode(1)
    # time.sleep(0.05)

    # spin() simply keeps python from exiting until this node is stopped
    # spin()只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()

if __name__ == "__main__":
    listener()
