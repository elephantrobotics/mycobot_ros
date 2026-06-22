#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
mycobot_topics.py
This ROS node manages the real-time state and control of a ultraARm robotic arm.
It publishes:
    - Joint angles
    - End-effector coordinates

It subscribes to:
    - Set joint angles
    - Set end-effector coordinates
    - Gripper status
    - Fresh mode status

This node uses threading and locking to ensure safe access to the robot over
serial communication.

Author: WangWeiJian
Date: 2025-11-24
"""

import time
import os
import sys
import signal
import threading
import traceback

import rospy
from std_msgs.msg import UInt8
from ultraarm_communication.msg import (
    MycobotAngles,
    MycobotCoords,
    MycobotSetAngles,
    MycobotSetCoords,
)
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

robot_msg = """
UltraArm P1 Status
--------------------------------
Joint Limit:
    joint 1: -165 ~ +165
    joint 2: -18 ~ +85
    joint 3: +90 ~ +200
    joint 4: -179 ~ +179
"""


class Watcher:
    """Watcher class handles KeyboardInterrupts in multithreaded Python programs.
    
    This approach prevents signals from being ignored in threads and ensures
    proper termination of child processes. Tested on Linux.
    """

    def __init__(self):
        """Forks a child process to monitor signals."""
        self.child = os.fork()
        if self.child == 0:
            return
        else:
            self.watch()

    def watch(self):
        """Wait for child process or handle KeyboardInterrupt to terminate child."""
        try:
            os.wait()
        except KeyboardInterrupt:
            rospy.loginfo("KeyboardInterrupt caught in Watcher")
            self.kill()
        sys.exit()

    def kill(self):
        """Kill the child process."""
        try:
            os.kill(self.child, signal.SIGKILL)
        except OSError:
            pass


class MycobotTopics:
    """ROS node class to manage MyCobot real-time topics."""

    def __init__(self):
        """Initialize ROS node, connect to robot, and prepare lock."""
        super(MycobotTopics, self).__init__()
        rospy.loginfo("Starting MyCobotTopics node...")
        rospy.init_node("mycobot_topics")
        port = rospy.get_param("~port", '/dev/ttyUSB0')
        baud = rospy.get_param("~baud", 1000000)
        rospy.loginfo("%s,%s" % (port, baud))
        self.mc = UltraArmP1(port, baud)
        self.lock = threading.Lock()
        self.output_robot_message()
        self.mc.set_joint_enable(0)
        time.sleep(0.05)

    def start(self):
        """Start all publisher and subscriber threads."""
        threads = [
            threading.Thread(target=self.pub_real_angles),
            threading.Thread(target=self.pub_real_coords),
            threading.Thread(target=self.sub_set_angles),
            threading.Thread(target=self.sub_set_coords),
            # threading.Thread(target=self.sub_gripper_status),
        ]

        for t in threads:
            t.setDaemon(True)
            t.start()

        for t in threads:
            t.join()

    def pub_real_angles(self):
        """Publish real joint angles to 'mycobot/angles_real' topic."""
        pub = rospy.Publisher("mycobot/angles_real", MycobotAngles, queue_size=5)
        ma = MycobotAngles()
        while not rospy.is_shutdown():
            with self.lock:
                try:
                    for i in range(3):
                        angles = self.mc.get_angles_info()
                        time.sleep(0.05)
                        if angles != -1:
                            break
                    if isinstance(angles, list) and len(angles) == 4 and all(c != -1 for c in angles):
                        ma.joint_1, ma.joint_2, ma.joint_3, ma.joint_4 = angles
                        pub.publish(ma)
                    else:
                        rospy.logwarn("Invalid angles received")
                except Exception:
                    e = traceback.format_exc()
                    rospy.logerr(f"SerialException: {e}")
            time.sleep(0.05)

    def pub_real_coords(self):
        """Publish real coordinates to 'mycobot/coords_real' topic."""
        pub = rospy.Publisher("mycobot/coords_real", MycobotCoords, queue_size=5)
        mc_msg = MycobotCoords()
        while not rospy.is_shutdown():
            with self.lock:
                try:
                    for i in range(3):
                        coords = self.mc.get_coords_info()
                        time.sleep(0.05)
                        if coords != -1:
                            break
                    if isinstance(coords, list) and len(coords) == 4 and all(c != -1 for c in coords):
                        mc_msg.x, mc_msg.y, mc_msg.z = coords[0], coords[1], coords[2]
                        mc_msg.rx = coords[3]
                        pub.publish(mc_msg)
                    else:
                        rospy.logwarn("Invalid coordinates received")
                except Exception:
                    e = traceback.format_exc()
                    rospy.logerr(f"SerialException: {e}")
            time.sleep(0.05)

    def sub_set_angles(self):
        """Subscribe to 'mycobot/angles_goal' to receive target angles."""
        def callback(data: MycobotSetAngles):
            angles = [
                data.joint_1, data.joint_2, data.joint_3,
                data.joint_4
            ]
            sp = int(data.speed)
            self.mc.set_angles(angles, sp, _async=False)

        rospy.Subscriber("mycobot/angles_goal", MycobotSetAngles, callback)
        rospy.spin()

    def sub_set_coords(self):
        """Subscribe to 'mycobot/coords_goal' to receive target coordinates."""
        def callback(data: MycobotSetCoords):
            coords = [data.x, data.y, data.z, data.rx]
            sp = int(data.speed)
            self.mc.set_coords(coords, sp, _async=False)

        rospy.Subscriber("mycobot/coords_goal", MycobotSetCoords, callback)
        rospy.spin()

    def output_robot_message(self):
        """Print robot joint limits and status information."""
        connect_status = False
        servo_infomation = "unknown"
        servo_temperature = "unknown"
        atom_version = "unknown"

        print(robot_msg)


if __name__ == "__main__":
    Watcher()
    mc_topics = MycobotTopics()
    mc_topics.start()
