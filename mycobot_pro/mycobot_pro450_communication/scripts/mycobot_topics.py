#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
mycobot_topics.py
This ROS node manages the real-time state and control of a Pro450 robotic arm.
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
Date: 2025-09-08
"""

import time
import os
import sys
import signal
import threading
import traceback

import rospy
from std_msgs.msg import UInt8
from mycobot_pro450_communication.msg import (
    MycobotAngles,
    MycobotCoords,
    MycobotSetAngles,
    MycobotSetCoords,
    MycobotGripperStatus,
    MycobotSetFreshMode,
    MycobotGetGripperValue,
)
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '4.0.1'

current_verison = pymycobot.__version__
print('Current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'The current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import Pro450Client


def read_joint5_firmware_limits(client):
    """Read J5's software limits from the connected controller."""
    try:
        lower = client.get_joint_min_angle(5)
        upper = client.get_joint_max_angle(5)
    except Exception as exc:
        rospy.logerr("Unable to read J5 firmware limits: %s", exc)
        return None
    if not isinstance(lower, (int, float)) or not isinstance(upper, (int, float)) or lower >= upper:
        rospy.logerr("Invalid J5 firmware limits returned by controller: %r, %r", lower, upper)
        return None
    rospy.loginfo("J5 firmware limits: [%.2f, %.2f] degrees", lower, upper)
    return lower, upper


def resolve_joint5_offset(limits, configured_offset):
    """Require the controller limits to match the selected URDF profile."""
    lower, upper = limits
    if abs(configured_offset) < 0.01:
        expected_lower, expected_upper = -162.0, 162.0
    elif abs(configured_offset - 90.0) < 0.01:
        expected_lower, expected_upper = -252.0, 72.0
    else:
        rospy.logerr("Unsupported J5 offset configured: %.2f degrees", configured_offset)
        return None
    if abs(lower - expected_lower) < 0.01 and abs(upper - expected_upper) < 0.01:
        return configured_offset
    rospy.logerr(
        "J5 firmware limits [%.2f, %.2f] do not match the selected URDF profile; expected [%.2f, %.2f]",
        lower, upper, expected_lower, expected_upper,
    )
    return None

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
            print("KeyboardInterrupt caught in Watcher")
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
        ip = rospy.get_param("~ip", '192.168.0.232')
        port = rospy.get_param("~port", 4500)
        rospy.loginfo("%s,%s" % (ip, port))
        self.mc = Pro450Client(ip, port)
        self.lock = threading.Lock()
        self.joint5_offset_deg = rospy.get_param(
            "/mycobot_pro_450/joint5_offset_deg", 0.0
        )
        if self.mc.is_power_on !=1:
            self.mc.power_on()
        time.sleep(0.05)
        if self.mc.get_fresh_mode()!=0:
            self.mc.set_fresh_mode(0)
        time.sleep(0.05)
        self.mc.set_limit_switch(2, 0)
        self.joint5_firmware_limits = read_joint5_firmware_limits(self.mc)
        if self.joint5_firmware_limits is not None:
            self.joint5_offset_deg = resolve_joint5_offset(
                self.joint5_firmware_limits, self.joint5_offset_deg
            )
        self.output_robot_message()

    def start(self):
        """Start all publisher and subscriber threads."""
        threads = [
            threading.Thread(target=self.pub_real_angles),
            threading.Thread(target=self.pub_real_coords),
            threading.Thread(target=self.sub_set_angles),
            threading.Thread(target=self.sub_set_firmware_angles),
            threading.Thread(target=self.sub_set_coords),
            threading.Thread(target=self.sub_gripper_status),
            threading.Thread(target=self.sub_fresh_mode_status),
            threading.Thread(target=self.sub_real_gripper_value),
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
                    angles = self.mc.get_angles()
                    if isinstance(angles, list) and len(angles) == 6 and all(c != -1 for c in angles):
                        if self.joint5_offset_deg is None:
                            rospy.logerr_once(
                                "Not publishing joint states because the J5 profile validation failed"
                            )
                            continue
                        logical_angles = list(angles)
                        logical_angles[4] += self.joint5_offset_deg
                        ma.joint_1, ma.joint_2, ma.joint_3, ma.joint_4, ma.joint_5, ma.joint_6 = logical_angles
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
                    coords = self.mc.get_coords()
                    if isinstance(coords, list) and len(coords) == 6 and all(c != -1 for c in coords):
                        mc_msg.x, mc_msg.y, mc_msg.z = coords[0], coords[1], coords[2]
                        mc_msg.rx, mc_msg.ry, mc_msg.rz = coords[3], coords[4], coords[5]
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
            if self.joint5_offset_deg is None:
                rospy.logerr("Refusing J5 command because the selected URDF profile is incompatible with firmware limits")
                return
            angles = [
                data.joint_1, data.joint_2, data.joint_3,
                data.joint_4, data.joint_5, data.joint_6
            ]
            angles[4] -= self.joint5_offset_deg
            if self.joint5_firmware_limits is None:
                rospy.logerr("Refusing J5 command because firmware limits are unavailable")
                return
            lower, upper = self.joint5_firmware_limits
            if not lower <= angles[4] <= upper:
                rospy.logerr(
                    "Refusing J5 command %.2f degrees outside firmware limits [%.2f, %.2f]",
                    angles[4], lower, upper,
                )
                return
            sp = int(data.speed)
            self.mc.send_angles(angles, sp)

        rospy.Subscriber("mycobot/angles_goal", MycobotSetAngles, callback)
        rospy.spin()

    def sub_set_coords(self):
        """Subscribe to 'mycobot/coords_goal' to receive target coordinates."""
        def callback(data: MycobotSetCoords):
            coords = [data.x, data.y, data.z, data.rx, data.ry, data.rz]
            sp = int(data.speed)
            self.mc.send_coords(coords, sp)

        rospy.Subscriber("mycobot/coords_goal", MycobotSetCoords, callback)
        rospy.spin()

    def sub_gripper_status(self):
        """Subscribe to 'mycobot/gripper_status' to open/close gripper."""
        def callback(data: MycobotGripperStatus):
            if data.Status:
                self.mc.set_pro_gripper_open()
            else:
                self.mc.set_pro_gripper_close()

        rospy.Subscriber("mycobot/gripper_status", MycobotGripperStatus, callback)
        rospy.spin()
        
    def sub_real_gripper_value(self):
        """Get Force Gripper Value"""
        pub = rospy.Publisher("mycobot/gripper_angle_real",
                              MycobotGetGripperValue, queue_size=5)
        ma = MycobotGetGripperValue()
        while not rospy.is_shutdown():
            with self.lock:
                try:
                    gripper_value = self.mc.get_pro_gripper_angle()
                    if gripper_value is not None:
                        ma.gripper_angle = gripper_value
                        pub.publish(ma)
                    else:
                        rospy.logwarn_throttle(2.0, "Gripper angle not available")
                except Exception as e:
                    rospy.logerr(f"SerialException: {e}")
            time.sleep(0.05)

    def sub_set_firmware_angles(self):
        """Accept explicit firmware angles for the fixed keyboard home pose."""
        def callback(data: MycobotSetAngles):
            angles = [
                data.joint_1, data.joint_2, data.joint_3,
                data.joint_4, data.joint_5, data.joint_6
            ]
            if self.joint5_firmware_limits is None:
                rospy.logerr("拒绝固件 J5 指令：无法读取固件限位 / Refusing firmware J5 command because firmware limits are unavailable")
                return
            lower, upper = self.joint5_firmware_limits
            if not lower <= angles[4] <= upper:
                rospy.logerr(
                    "拒绝固件 J5 指令 %.2f 度：超出固件限位 [%.2f, %.2f] / Refusing firmware J5 command outside firmware limits",
                    angles[4], lower, upper,
                )
                return
            self.mc.send_angles(angles, int(data.speed))

        rospy.Subscriber("mycobot/firmware_angles_goal", MycobotSetAngles, callback)
        rospy.spin()


    def sub_fresh_mode_status(self):
        """Subscribe to 'mycobot/fresh_mode_status' to switch fresh mode."""
        def callback(data: MycobotSetFreshMode):
            self.mc.set_fresh_mode(1 if data.Status == 1 else 0)

        rospy.Subscriber("mycobot/fresh_mode_status", MycobotSetFreshMode, callback)
        rospy.spin()

    def output_robot_message(self):
        """Print the six joint limits read from the connected controller."""
        lines = ["", "MyCobot Status", "--------------------------------", "Joint Limit:"]
        for joint_id in range(1, 7):
            try:
                lower = self.mc.get_joint_min_angle(joint_id)
                upper = self.mc.get_joint_max_angle(joint_id)
                if not isinstance(lower, (int, float)) or not isinstance(upper, (int, float)):
                    raise ValueError("controller returned a non-numeric limit")
                lines.append(
                    "    joint {}: {:+g} ~ {:+g}".format(joint_id, lower, upper)
                )
            except Exception as exc:
                lines.append("    joint {}: unavailable ({})".format(joint_id, exc))
        print("\n".join(lines))


if __name__ == "__main__":
    Watcher()
    mc_topics = MycobotTopics()
    mc_topics.start()
