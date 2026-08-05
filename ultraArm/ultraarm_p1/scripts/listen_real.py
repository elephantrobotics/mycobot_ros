#!/usr/bin/env python3
# encoding:utf-8
import time
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from rospy import ServiceException
from ultraarm_communication.srv import GetAngles
from pymycobot.robot_info import RobotLimit

ROBOT_LIMIT = RobotLimit.robot_limit.get("UltraArmP1", {})
JOINT_LIMITS = list(zip(
    ROBOT_LIMIT.get("angles_min", [-165, -18, 89, -179]),
    ROBOT_LIMIT.get("angles_max", [165, 85, 200, 179]),
))


def valid_angles(angles):
    """Return True if all joint angles are inside the expected P1 range."""
    return all(low <= angle <= high for angle, (low, high) in zip(angles, JOINT_LIMITS))


def talker():
    rospy.loginfo("start ...")
    rospy.init_node("real_listener", anonymous=True)
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    publish_rate = max(float(rospy.get_param("~publish_rate", 10.0)), 0.1)
    rate = rospy.Rate(publish_rate)
    # rate = rospy.Rate(30)  # 30hz

    # pub joint state
    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = ["J1", "J2", "J3", "J4"]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    # waiting util server `get_joint_angles` enable
    rospy.loginfo("wait service")
    rospy.wait_for_service("get_joint_angles")
    func = rospy.ServiceProxy("get_joint_angles", GetAngles)

    rospy.loginfo("start loop ...")
    while not rospy.is_shutdown():
        
        # get real angles from server
        try:
            res = func()
        except ServiceException as exc:
            if rospy.is_shutdown():
                break
            # rospy.logwarn_throttle(2.0, "Failed to get joint angles: %s", exc)
            rate.sleep()
            continue
        
        if res is None:
            continue
        angles = [res.joint_1, res.joint_2, res.joint_3, res.joint_4]
        if not valid_angles(angles):
            # rospy.logwarn_throttle(5.0, "Skip invalid joint angles for RViz: %s", angles)
            rate.sleep()
            continue
        radians_list = [
            angles[0] * (math.pi / 180),
            angles[1] * (math.pi / 180),
            (angles[2] - 90) * (math.pi / 180),
            angles[3] * (math.pi / 180),
        ]
        # rospy.loginfo("res: {}".format(radians_list))

        # publish angles
        joint_state_send.header.stamp = rospy.Time.now()
        joint_state_send.position = radians_list
        pub.publish(joint_state_send)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
