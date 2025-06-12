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
import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
import time
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
from pymycobot.mycobot import MyCobot
from pymycobot import MyCobot280  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot320初始化

mcs = MyCobot280(PI_PORT, 115200)
mc, pub_arm_command, pub_gripper_command = None, None, None
robot_joint_pose = []

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

    angles = mc.get_angles()
    gripper_ang = mcs.get_gripper_value()

    print(gripper_ang)
    data_list = []
    for index, value in enumerate(angles):
        data_list.append(value * (math.pi / 180))
    
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
    arm_point.positions = data_list  # Convert back to radians for the command
    arm_point.velocities = [0.0] * expected_num_arm_joints  # Initialize velocities to zero
    arm_point.accelerations = [0.0] * expected_num_arm_joints  # Initialize accelerations to zero
    arm_point.time_from_start = rospy.Duration(0.5)
    arm_joint_trajectory.points.append(arm_point)

    pub_arm_command.publish(arm_joint_trajectory)  # Publish the arm joint trajectory

    # Map actual MyCobot gripper angle to Gazebo gripper position
    mycobot_min_angle = 3
    mycobot_max_angle = 91
    gazebo_min_position = -0.68
    gazebo_max_position = 0.15

    def map_mycobot_to_gazebo(mycobot_angle):
        return ((mycobot_angle - mycobot_min_angle) / (mycobot_max_angle - mycobot_min_angle)) * \
               (gazebo_max_position - gazebo_min_position) + gazebo_min_position

    # Assuming gripper_ang contains the angle from the real MyCobot
    mapped_gripper_position = map_mycobot_to_gazebo(gripper_ang)

    # Create and publish gripper joint trajectory
    gripper_joint_trajectory = JointTrajectory()
    gripper_joint_trajectory.header.stamp = rospy.Time.now()
    gripper_joint_trajectory.joint_names = expected_gripper_joint_names
    gripper_point = JointTrajectoryPoint()
    gripper_point.positions = [mapped_gripper_position]  # No need to convert to radians here
    gripper_point.velocities = [0.0] * expected_num_gripper_joints  # Initialize velocities to zero
    gripper_point.accelerations = [0.0] * expected_num_gripper_joints  # Initialize accelerations to zero
    gripper_point.time_from_start = rospy.Duration(0.5)
    gripper_joint_trajectory.points.append(gripper_point)

    pub_gripper_command.publish(gripper_joint_trajectory)  # Publish the gripper joint trajectory


def listener():
    global mc, pub_arm_command, pub_gripper_command

    rospy.init_node("control_slider", anonymous=True)
    print("Try connect real mycobot...")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    print("port: {}, baud: {}\n".format(port, baud))
    try:
        from pymycobot.mycobot import MyCobot
        mc = MyCobot(port, baud)
    except Exception as e:
        print(e)
        print(
            """\
            \rTry connect mycobot failed!
            \rPlease check whether connected with mycobot.
            \rPlease check whether the port or baud is right.
        """
        )
        exit(1)
    mc.release_all_servos()
    time.sleep(0.1)
    print("Release all servos over.\n")

    pub_arm_command = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
    pub_gripper_command = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, callback)

    print("spin ...")
    rospy.spin()

if __name__ == "__main__":
    listener()



