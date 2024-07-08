#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import time
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot.mycobot import MyCobot

# Global Variables
joint_names = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

def talker():
    rospy.init_node("display", anonymous=True)

    print("Try connect real mycobot...")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    print("port: {}, baud: {}\n".format(port, baud))
    try:
        mycobot = MyCobot(port, baud)
    except Exception as e:
        print(e)
        print(
            """\
            \rTry connect mycobot failed!
            \rPlease check wether connected with mycobot.
            \rPlease chckt wether the port or baud is right.
        """
        )
        exit(1)
    mycobot.release_all_servos()
    time.sleep(0.1)
    print("Rlease all servos over.\n")

    pub_command = rospy.Publisher("/mycobot_position_controller/command", JointTrajectory, queue_size=10)
    rate = rospy.Rate(30)  # 30hz

    # pub joint state

    print("publishing ...")
    while not rospy.is_shutdown():

        angles = mycobot.get_radians()
        data_list = []
        for index, value in enumerate(angles):
            data_list.append(value)

        # rospy.loginfo('{}'.format(data_list))

        # Create JointTrajectory message
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = data_list
        point.velocities = [0.0] * len(joint_names)
        point.time_from_start = rospy.Duration(0.1)  # Set a small duration to continuously update

        traj_msg.points.append(point)
        pub_command.publish(traj_msg)

        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
