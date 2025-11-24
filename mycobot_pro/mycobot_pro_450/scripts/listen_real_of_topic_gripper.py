#!/usr/bin/env python3
# encoding:utf-8

import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_pro450_communication.msg import MycobotAngles, MycobotGetGripperValue


class Listener(object):
    def __init__(self):
        super(Listener, self).__init__()

        rospy.loginfo("start ...")
        rospy.init_node("real_listener_topic_gripper", anonymous=True)
        # init publisher
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        # init subscriber
        self.sub = rospy.Subscriber("mycobot/angles_real", MycobotAngles, self.callback)
        self.sub_gripper = rospy.Subscriber("mycobot/gripper_angle_real", MycobotGetGripperValue, self.gripper_callback)
        self.gripper_angle = None
        rospy.spin()
        

    def gripper_callback(self, data):
        """Force Gripper angle update"""
        if data.gripper_angle >= 0:
            self.gripper_angle = data.gripper_angle
        else:
            self.gripper_angle = 1
            rospy.logwarn("Unable to read the gripper angle normally: {}".format(self.gripper_angle))

    def callback(self, data):
        """`mycobot/angles_real` subscriber callback method.

        Args:
            data (MycobotAngles): callback argument.
        """
        # ini publisher object
        joint_state_send = JointState()
        joint_state_send.header = Header()

        joint_state_send.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_controller"]
        joint_state_send.velocity = [0]
        joint_state_send.effort = []
        joint_state_send.header.stamp = rospy.Time.now()

        # process callback data
        radians_list = [
            data.joint_1 * (math.pi / 180),
            data.joint_2 * (math.pi / 180),
            data.joint_3 * (math.pi / 180),
            data.joint_4 * (math.pi / 180),
            data.joint_5 * (math.pi / 180),
            data.joint_6 * (math.pi / 180),
            self.gripper_angle * (math.pi / 180),
        ]
        # rospy.loginfo("res: {}".format(radians_list))

        joint_state_send.position = radians_list
        self.pub.publish(joint_state_send)


if __name__ == "__main__":
    try:
        Listener()
    except rospy.ROSInterruptException:
        pass
