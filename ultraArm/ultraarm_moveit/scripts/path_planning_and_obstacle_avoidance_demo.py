#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslib, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MoveItPlanningDemo:
    def __init__(self):
        # API to initialize move_group. 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node,初始化ROS节点
        rospy.init_node("moveit_ik_demo")

        # Initialize the scene object to listen for changes in the external environment
        # 初始化场景对象，用来监听外部环境的变化
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)

        # Initialize self.arm group in the robotic arm that needs to be controlled by move group 
        # 初始化需要使用move group控制的机械臂中的self.arm group
        self.arm = moveit_commander.MoveGroupCommander("arm_group")

        # Get the name of the terminal link. 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference coordinate system used for the target position.
        #  设置目标位置所使用的参考坐标系
        self.reference_frame = "joint1"
        self.arm.set_pose_reference_frame(self.reference_frame)

        # Allow replanning when motion planning fails. 
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)

        # Set the allowable error of position (unit: meter) and attitude (unit: radian). 
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)

    def moving(self):
        # Control the robotic arm to return to the initialization position first. 控制机械臂先回到初始化位置
        self.arm.set_named_target("init_pose")
        self.arm.go()
        rospy.sleep(2)

        # Set the target pose in the robotic arm workspace, the position is described by x, y, z coordinates,
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # Pose is described by quaternion, based on base_link coordinate system 
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.132
        target_pose.pose.position.y = -0.150
        target_pose.pose.position.z = 0.075
        target_pose.pose.orientation.x = 0.026
        target_pose.pose.orientation.y = 1.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.014

        # Set the current state of the robot arm as the initial state of motion. 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        # Set the target pose of the terminal motion of the robotic arm. 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose, self.end_effector_link)

        traj = self.arm.plan()

        # Control the motion of the robotic arm according to the planned motion path. 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj)
        rospy.sleep(1)

        # Control the terminal of the robotic arm to move 5cm to the right. Parameter 1 represents y, 0,1,2,3,4,5 represents xyzrpy
        # 控制机械臂终端向右移动5cm 參数1是代表y， 0,1,2,3,4,5 代表xyzrpy
        self.arm.shift_pose_target(1, 0.12, self.end_effector_link)
        self.arm.go()
        rospy.sleep(1)

        self.arm.shift_pose_target(1, 0.1, self.end_effector_link)
        self.arm.go()
        rospy.sleep(1)

        # Control the end of the robotic arm to rotate 90 degrees in reverse, 0,1,2,3,4,5 represents xyzrpy
        # 控制机械臂终端反向旋转90度,  0,1,2,3,4,5 代表xyzrpy
        # self.arm.shift_pose_target(3, -1.57, end_effector_link)
        # self.arm.go()
        # rospy.sleep(1)

    def run(self):
        self.scene.remove_world_object("suit")

        self.moving()

        # Add the environment. 添加环境
        quat = quaternion_from_euler(3.1415, 0, -1.57)

        suit_post = PoseStamped()
        suit_post.header.frame_id = self.reference_frame
        suit_post.pose.position.x = 0.0
        suit_post.pose.position.y = 0.0
        suit_post.pose.position.z = -0.02
        suit_post.pose.orientation.x = quat[0]
        suit_post.pose.orientation.y = quat[1]
        suit_post.pose.orientation.z = quat[2]
        suit_post.pose.orientation.w = quat[3]

        suit_path = (
            roslib.packages.get_pkg_dir("mycobot_description")
            + "/urdf/mycobot/suit_env.dae"
        )
        # need `pyassimp==3.3`
        self.scene.add_mesh("suit", suit_post, suit_path)
        rospy.sleep(2)

        # Run again if there is environmental impact. 有环境影响后再运行一次
        self.moving()

        # Close and exit moveit. 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    o = MoveItPlanningDemo()
    o.run()
