#!/usr/bin/env python3

"""[summary]
sync_plan.py
This ROS node subscribes to JointState messages and sends the corresponding
angles to a Pro450 robotic arm using pymycobot.

It converts radians from JointState into degrees and publishes them
to the robot through the UltraArmP1.

Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyUSB0'
    baud: serial prot baudrate. Defaults is 115200.
    

Author: WangWeiJian
Date: 2025-11-24
"""

import rospy
from sensor_msgs.msg import JointState

import math
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '4.0.5'

current_verison = pymycobot.__version__
rospy.loginfo('Current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'The current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
else:
    rospy.loginfo('pymycobot library version meets the requirements!')
    from pymycobot import UltraArmP1

ua = None
last_invalid_pair = None
last_sent_angles = None
was_invalid = False

J2_RANGE = (-18.0, 85.0)
J3_RANGE = (-1.0, 110.0)
ZERO_EPS_DEG = 0.1
INVALID_WARN_DELTA_DEG = 0.5
SEND_ANGLE_DELTA_DEG = 0.2
speed = 25


def snap_zero(angle_deg):
    return 0.0 if abs(angle_deg) < ZERO_EPS_DEG else angle_deg


def valid_region(j2_deg, j3_deg):
    """Return whether the J2/J3 combination is mechanically reachable."""

    if not (J2_RANGE[0] <= j2_deg <= J2_RANGE[1] and J3_RANGE[0] <= j3_deg <= J3_RANGE[1]):
        return False

    if -18 <= j2_deg < 0:
        # J2 < 0: reject J3 >= 42° (same hard cut as joint_coupling_node)
        if j3_deg >= 42.0:
            return False
        cond1 = math.cos(math.radians(-j2_deg + j3_deg)) - math.sin(math.radians(45 + j2_deg)) <= 7 / 30
        cond2 = abs(math.cos(math.radians(-j2_deg + j3_deg))) >= 15.4 / 30
        return cond1 and cond2

    if 0 <= j2_deg <= 50.87:
        return math.cos(math.radians(j2_deg - j3_deg)) >= 15.4 / 30

    if 50.87 < j2_deg < 76.72:
        return True

    if 76.72 <= j2_deg <= 85:
        return abs(math.cos(math.radians(j2_deg - j3_deg))) >= 6.89 / 30

    return False


def joint_angle_deg(msg, joint_name):
    try:
        index = msg.name.index(joint_name)
    except ValueError:
        raise KeyError(joint_name)

    return round(snap_zero(math.degrees(msg.position[index])), 2)


def callback(data):
    global last_invalid_pair, last_sent_angles, was_invalid

    """Callback function for ROS JointState subscription.

    This function converts incoming joint positions (radians) to angles
    in degrees and sends them to the ultarArm P1 robotic arm.

    Args:
        data (JointState): Joint state message containing joint positions.
    """
    try:
        joint1 = joint_angle_deg(data, "J1")
        joint2 = joint_angle_deg(data, "J2")
        joint3 = joint_angle_deg(data, "J3")
        joint4 = joint_angle_deg(data, "J4")
    except (KeyError, IndexError):
        rospy.logwarn_throttle(2.0, "JointState missing one of J1/J2/J3/J4; message ignored")
        return

    if not valid_region(joint2, joint3):
        should_warn = not was_invalid
        if last_invalid_pair is not None:
            should_warn = should_warn or abs(joint2 - last_invalid_pair[0]) >= INVALID_WARN_DELTA_DEG
            should_warn = should_warn or abs(joint3 - last_invalid_pair[1]) >= INVALID_WARN_DELTA_DEG
        else:
            should_warn = True

        if should_warn:
            rospy.logwarn(
                "Rejected unsafe J2/J3 combination from MoveIt: J2=%.2f deg, J3=%.2f deg",
                joint2,
                joint3,
            )
            last_invalid_pair = (joint2, joint3)
        was_invalid = True
        return

    was_invalid = False
    last_invalid_pair = None
    joint3 = joint3 + 90
    angles_list = [joint1, joint2, joint3, joint4]
    angles_list = [round(angle, 2) for angle in [joint1, joint2, joint3, joint4]]

    if last_sent_angles is not None:
        max_delta = max(abs(current - previous) for current, previous in zip(angles_list, last_sent_angles))
        if max_delta < SEND_ANGLE_DELTA_DEG:
            return

    last_sent_angles = list(angles_list)
    rospy.loginfo("send angles: %s", angles_list)
    ua.set_angles(angles_list, speed, _async=False)


def listener():
    global ua, speed
    rospy.init_node("control_slider", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyUSB0") # Select connected device. 选择连接设备
    baud = rospy.get_param("~baud", 1000000)
    speed = int(rospy.get_param("~speed", 25))
    print(port, baud)
    ua = UltraArmP1(port, baud)
    ua.set_joint_enable(0)
    
    rospy.Subscriber("joint_states", JointState, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()