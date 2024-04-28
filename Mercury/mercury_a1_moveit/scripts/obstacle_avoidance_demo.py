#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf


class MoveItPlanningDemo:
    def __init__(self):
        rospy.init_node('moveit_avoid_obstacles', anonymous=True)
        # 初始化MoveIt
        moveit_commander.roscpp_initialize(sys.argv)

        # 创建RobotCommander对象
        self.robot = moveit_commander.RobotCommander()

        # 创建PlanningSceneInterface对象
        self.scene = moveit_commander.PlanningSceneInterface()

        # 创建MoveGroupCommander对象
        self.arm_group = moveit_commander.MoveGroupCommander("arm_group")
        
        # 获取末端关节的名称
        self.end_effector_link = self.arm_group.get_end_effector_link()
        
        # 设置目标位置所使用的坐标参考系
        self.reference_frame = 'base'
        self.arm_group.set_pose_reference_frame(self.reference_frame)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm_group.set_goal_position_tolerance(0.01)
        self.arm_group.set_goal_orientation_tolerance(0.05)
        
        # 当运动规划失败后，允许重新规划
        self.arm_group.allow_replanning(True)
        # 设置规划的最大时间为20秒
        self.arm_group.set_planning_time(20)
        # 设置规划尝试次数为10次（或者更大的值）
        self.arm_group.set_num_planning_attempts(20)  

        
    def add_scene(self):
        # 添加第一个圆柱作为障碍物（垂直于平面）
        cylinder1_pose = geometry_msgs.msg.PoseStamped()
        cylinder1_pose.header.frame_id = self.robot.get_planning_frame()
        cylinder1_pose.pose.position.x = 0.15
        cylinder1_pose.pose.position.y = 0
        cylinder1_pose.pose.position.z = 0.30
        cylinder1_pose.pose.orientation.w = 1.0
        self.scene.add_cylinder("cylinder1", cylinder1_pose, height=0.6, radius=0.01)

        # 添加第二个圆柱作为障碍物（水平于平面，构成十字架）
        cylinder2_pose = geometry_msgs.msg.PoseStamped()
        cylinder2_pose.header.frame_id = self.robot.get_planning_frame()
        cylinder2_pose.pose.position.x = 0.15
        cylinder2_pose.pose.position.y = 0
        cylinder2_pose.pose.position.z = 0.40
        cylinder2_pose.pose.orientation.w = 1.0
        cylinder2_pose.pose.orientation.x = 0.707  # 围绕x轴旋转90度（水平方向）
        cylinder2_pose.pose.orientation.y = 0.0
        cylinder2_pose.pose.orientation.z = 0.0
        cylinder2_pose.pose.orientation.w = 0.707
        self.scene.add_cylinder("cylinder2", cylinder2_pose, height=0.6, radius=0.02)
        # 发布当前场景信息
        planning_scene = moveit_msgs.msg.PlanningScene()
        planning_scene.world.collision_objects.extend(self.scene.get_objects().values())
        planning_scene.is_diff = True

        planning_scene_publisher = rospy.Publisher('/planning_scene', moveit_msgs.msg.PlanningScene, queue_size=1)
        planning_scene_publisher.publish(planning_scene)
        rospy.sleep(2)

    def robot_move(self):

        # 控制机械臂回到初始化位置
        self.arm_group.set_named_target('init_pose')
        self.arm_group.go()
        rospy.sleep(3)

        
        # 设置机械臂运动的目标点，使用笛卡尔空间坐标位置表示（单位：米），姿态使用四元数表示
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.142  # 设置目标点的x坐标
        target_pose.pose.position.y = -0.140  # 设置目标点的y坐标
        target_pose.pose.position.z = 0.075  # 设置目标点的z坐标
        target_pose.pose.orientation.x = 0.026
        target_pose.pose.orientation.y = 1.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.014
        
        # 更新当前的位姿
        self.arm_group.set_start_state_to_current_state()

        # 获取机械臂当前的关节状态
        current_joint_values = self.arm_group.get_current_joint_values()
        
        # 打印当前关节状态
        print("Current Joint Values:", current_joint_values)
        print("end Joint Values:", self.end_effector_link)
        # 设置机械臂的目标姿态
        self.arm_group.set_pose_target(target_pose, self.end_effector_link)
        # 进行运动规划
        plan = self.arm_group.plan()
        # print('plan point:', plan[1])
        # 执行运动
        self.arm_group.execute(plan[1])
        rospy.sleep(3)
        # 获取末端执行器的姿态
        end_effector_pose = self.arm_group.get_current_pose().pose

        # 打印末端执行器的坐标位置
        print("End Effector Position:", end_effector_pose.position)
        print("End Effector Orientation:", end_effector_pose.orientation)
        # 控制机械臂末端向右移动5cm 參數1是代表y， 0,1,2,3,4,5 代表xyzrpy
        # self.arm_group.shift_pose_target(1, 0.22, self.end_effector_link)
        # self.arm_group.go()
        # rospy.sleep(5)
    
        # 设置机械臂的目标位置，使用7轴的位置数据进行描述（单位：弧度）
        # joint_pose = [0.2967, 0, 0, -1.57000, 0, -1.3439, 0]
        # joint_pose = [0.2967, 0, 0, 0, 0, -1.3439, 0]
        # arm_group.set_joint_value_target(joint_pose)
      
        # 控制机械臂完成运动
        # arm_group.go()
        # rospy.sleep(10)
        # 控制机械臂回到初始化位置
        # arm_group.set_named_target('init_pose')
        # arm_group.go()
        
    def run(self):
        # 移除所有障碍物
        # self.scene.remove_world_object("cylinder1")
        # self.scene.remove_world_object("cylinder2")
        # 没有障碍物运行一次
        # self.robot_move()
        
        # 增加障碍物
        self.add_scene()
        rospy.sleep(3)
        # 获取当前场景中的所有障碍物
        current_obstacles = self.scene.get_known_object_names()
        rospy.loginfo("Current obstacles in the scene: %s", current_obstacles)
        rospy.sleep(2)
        # 有障碍物后再运行一次
        self.robot_move()
        # 关闭MoveIt
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        

if __name__ == '__main__':
    try:
        obstacle = MoveItPlanningDemo()
        obstacle.run()
    except rospy.ROSInterruptException:
        pass
