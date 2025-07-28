#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

class GroundAdder:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('add_ground_node', anonymous=True)
        
        # 初始化 PlanningSceneInterface
        self.scene = PlanningSceneInterface()
        rospy.sleep(2)  # 等待场景初始化

    def add_ground(self):
        # 定义地面的姿态
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = "base"  # 基座坐标系
        ground_pose.pose.position.x = 0.0
        ground_pose.pose.position.y = 0.0
        ground_pose.pose.position.z = -0.01  # 地面位于基座下方 0.01 米
        ground_pose.pose.orientation.w = 1.0

        # 添加地面到场景中
        self.scene.add_box("ground", ground_pose, size=(1.5, 1.5, 0.005))
        rospy.loginfo("Ground added to the planning scene.")

if __name__ == "__main__":
    try:
        ground_adder = GroundAdder()
        ground_adder.add_ground()
        rospy.spin()  # 保持节点运行，防止退出
    except rospy.ROSInterruptException:
        pass
