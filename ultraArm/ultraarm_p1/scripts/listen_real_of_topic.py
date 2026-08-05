#!/usr/bin/env python3
# encoding:utf-8

import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot.robot_info import RobotLimit
from ultraarm_communication.msg import MycobotAngles

ROBOT_LIMIT = RobotLimit.robot_limit.get("UltraArmP1", {})
JOINT_LIMITS = list(zip(
    ROBOT_LIMIT.get("angles_min", [-165, -18, 89, -179]),
    ROBOT_LIMIT.get("angles_max", [165, 85, 200, 179]),
))


def valid_angles(angles):
    """Return True if all joint angles are inside the expected P1 range."""
    return all(low <= angle <= high for angle, (low, high) in zip(angles, JOINT_LIMITS))


class Listener(object):
    def __init__(self):
        super(Listener, self).__init__()

        rospy.loginfo("start ...")
        rospy.init_node("real_listener_topic", anonymous=True)
        # init publisher
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        # init subscriber
        self.sub = rospy.Subscriber("mycobot/angles_real", MycobotAngles, self.callback)
        rospy.spin()

    def callback(self, data):
        """`mycobot/angles_real` subscriber callback method.

        Args:
            data (MycobotAngles): callback argument.
        """
        # ini publisher object
        joint_state_send = JointState()
        joint_state_send.header = Header()

        joint_state_send.name = ["J1", "J2", "J3", "J4"]
        joint_state_send.velocity = [0]
        joint_state_send.effort = []
        joint_state_send.header.stamp = rospy.Time.now()

        angles = [data.joint_1, data.joint_2, data.joint_3, data.joint_4]
        if not valid_angles(angles):
            # rospy.logwarn_throttle(5.0, "Skip invalid joint angles for RViz: %s", angles)
            return

        # process callback data
        radians_list = [
            angles[0] * (math.pi / 180),
            angles[1] * (math.pi / 180),
            (angles[2] - 90) * (math.pi / 180),
            angles[3] * (math.pi / 180),
        ]
        # rospy.loginfo("res: {}".format(radians_list))

        joint_state_send.position = radians_list
        self.pub.publish(joint_state_send)


if __name__ == "__main__":
    try:
        Listener()
    except rospy.ROSInterruptException:
        pass
