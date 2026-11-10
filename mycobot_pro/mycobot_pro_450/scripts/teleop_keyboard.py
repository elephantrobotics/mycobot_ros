#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
teleop_keyboard.py
Keyboard teleoperation for MyCobot Pro450 using ROS1 topics.

F100 gripper keys (g/h) are enabled automatically when robot_description
contains gripper_controller (e.g. teleop_keyboard.launch gripper:=f100).

Author: WangWeiJian
Date: 2025-09-08
"""

from __future__ import print_function
import sys
import time
import termios
import tty
import rospy
from mycobot_pro450_communication.msg import (
    MycobotAngles,
    MycobotCoords,
    MycobotSetAngles,
    MycobotSetCoords,
    MycobotGripperStatus,
)
from pymycobot.robot_info import RobotLimit

MSG_BASE = """\
Mycobot Pro450 Teleop Keyboard Controller (ROS1 - Topic Version)
---------------------------------------------------------
Movement (Cartesian):
              w (x+)
    a (y+)    s (x-)    d (y-)
              z (z-)    x (z+)

Rotation (Euler angles):
    u (rx+)  i (ry+)  o (rz+)
    j (rx-)  k (ry-)  l (rz-)

Movement Step:
    + : Increase movement step size
    - : Decrease movement step size
"""

MSG_GRIPPER = """\
Gripper:
    g - open    h - close
"""

MSG_OTHER = """\
Other:
    1 - Go to init pose
    2 - Go to home pose
    3 - Save current pose as home
    q - Quit
"""

COORD_AXES = ('x', 'y', 'z', 'rx', 'ry', 'rz')


def detect_f100_gripper():
    """Return True when the loaded URDF exposes gripper_controller."""
    if rospy.has_param("robot_description"):
        return "gripper_controller" in rospy.get_param("robot_description")
    return False


def get_coordinate_limits(joint5_offset_deg):
    """Load Cartesian limits from pymycobot's Pro450 profile."""
    profiles = RobotLimit.robot_limit

    if abs(joint5_offset_deg - 90.0) < 0.01:
        profile = "Pro450ClientNew"
        if profile not in profiles:
            raise RuntimeError(
                "Current pymycobot version does not support "
                "Pro450ClientNew. Please upgrade pymycobot to "
                "version 4.0.8 or later."
            )
    else:
        profile = "Pro450Client"
    try:
        limits = profiles[profile]
        return dict(zip(COORD_AXES, zip(limits["coords_min"], limits["coords_max"])))
    except (AttributeError, KeyError, TypeError) as exc:
        raise RuntimeError(
            "Unable to load coordinate limits for {}: {}".format(profile, exc)
        )


def vels(speed: float, turn: float) -> str:
    """Return formatted speed and change percent info."""
    return "currently:\tspeed: {}\tchange percent: {}".format(speed, turn)


class Raw:
    """Context manager for reading raw input from terminal."""

    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)


class MycobotTeleopTopic:
    """Keyboard teleoperation node for MyCobot Pro450 using ROS topics."""

    def __init__(self):
        """Initialize ROS node, parameters, publishers, and subscribers."""
        rospy.init_node("teleop_keyboard_topic")

        self.joint5_offset_deg = rospy.get_param(
            "/mycobot_pro_450/joint5_offset_deg", 0.0
        )
        self.coord_limits = get_coordinate_limits(self.joint5_offset_deg)
        self.gripper_enabled = detect_f100_gripper()

        self.speed = rospy.get_param("~speed", 50)
        self.change_percent = rospy.get_param("~change_percent", 5)
        self.change_len = 250 * self.change_percent / 100.0
        self.change_angle = 180 * self.change_percent / 100.0

        self.curr_coords = [0] * 6
        self.curr_angles = [0] * 6
        self.record_coords = None
        # home_pose is always stored in firmware coordinates for key 2.
        # Default factory pose uses firmware J5 = 0°.
        self.home_pose = [0, -10, -123, 45, 0, 0]
        self.ready_for_coords = False

        rospy.Subscriber("mycobot/coords_real", MycobotCoords, self.coords_callback)
        rospy.Subscriber("mycobot/angles_real", MycobotAngles, self.angles_callback)

        self.coords_pub = rospy.Publisher("mycobot/coords_goal", MycobotSetCoords, queue_size=1)
        self.angles_pub = rospy.Publisher("mycobot/angles_goal", MycobotSetAngles, queue_size=1)
        self.firmware_angles_pub = rospy.Publisher(
            "mycobot/firmware_angles_goal", MycobotSetAngles, queue_size=1
        )
        self.gripper_pub = None
        if self.gripper_enabled:
            self.gripper_pub = rospy.Publisher(
                "mycobot/gripper_status", MycobotGripperStatus, queue_size=1
            )
            rospy.loginfo("F100 gripper keys enabled (g open / h close)")
        else:
            rospy.loginfo("No gripper_controller in robot_description; gripper keys disabled")

        rospy.loginfo("Waiting to receive current coordinates...")
        while self.curr_coords == [0] * 6 and not rospy.is_shutdown():
            time.sleep(0.1)

        self.record_coords = list(self.curr_coords)
        help_msg = MSG_BASE
        if self.gripper_enabled:
            help_msg += MSG_GRIPPER
        help_msg += MSG_OTHER
        print(help_msg)
        print(vels(self.speed, self.change_percent))
        rospy.loginfo(
            "Current moving step: position %.1f mm, angle attitude %.1f°",
            self.change_len,
            self.change_angle,
        )

    def coords_callback(self, msg: MycobotCoords):
        """Update current coordinates from ROS topic."""
        self.curr_coords = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]

    def angles_callback(self, msg: MycobotAngles):
        """Update current logical angles from ROS topic."""
        self.curr_angles = [
            msg.joint_1, msg.joint_2, msg.joint_3,
            msg.joint_4, msg.joint_5, msg.joint_6,
        ]

    def logical_to_firmware_angles(self, logical_angles):
        """Convert angles_real (logical) joints to firmware coordinates."""
        firmware = [round(float(v), 2) for v in logical_angles[:6]]
        firmware[4] = round(firmware[4] - self.joint5_offset_deg, 2)
        return firmware

    @staticmethod
    def format_angles(angles):
        """Format joint angles for logs with two decimal places."""
        return [round(float(v), 2) for v in angles[:6]]

    def firmware_j5_limits(self):
        """Expected firmware J5 software limits for the selected URDF profile."""
        if abs(self.joint5_offset_deg - 90.0) < 0.01:
            return -252.0, 72.0
        return -162.0, 162.0

    def send_coords(self):
        """Publish current target coordinates to robot."""
        goal = MycobotSetCoords()
        goal.x, goal.y, goal.z, goal.rx, goal.ry, goal.rz = self.record_coords
        goal.speed = self.speed
        self.coords_pub.publish(goal)

    def send_angles(self, angles):
        """Publish target logical joint angles to robot."""
        goal = MycobotSetAngles()
        goal.joint_1, goal.joint_2, goal.joint_3, goal.joint_4, goal.joint_5, goal.joint_6 = angles
        goal.speed = self.speed
        self.angles_pub.publish(goal)

    def send_firmware_angles(self, angles):
        """Publish a firmware-coordinate joint command (used by key 2 home)."""
        goal = MycobotSetAngles()
        goal.joint_1, goal.joint_2, goal.joint_3, goal.joint_4, goal.joint_5, goal.joint_6 = angles
        goal.speed = self.speed
        self.firmware_angles_pub.publish(goal)

    def run(self):
        """Main loop to read keyboard input and control the robot."""
        while not rospy.is_shutdown():
            try:
                with Raw(sys.stdin):
                    key = sys.stdin.read(1)
            except Exception:
                continue

            try:
                if key == 'q':
                    break

                elif key == '+':
                    self.change_percent = min(self.change_percent + 1, 20)
                    self.change_angle = 180 * self.change_percent / 100.0
                    self.change_len = 250 * self.change_percent / 100.0
                    rospy.loginfo(
                        "Increase change_percent to %d%%, move step: %.1f mm, angle step: %.1f°",
                        self.change_percent, self.change_len, self.change_angle,
                    )
                elif key == '-':
                    self.change_percent = max(self.change_percent - 1, 1)
                    self.change_angle = 180 * self.change_percent / 100.0
                    self.change_len = 250 * self.change_percent / 100.0
                    rospy.loginfo(
                        "Decrease change_percent to %d%%, move step: %.1f mm, angle step: %.1f°",
                        self.change_percent, self.change_len, self.change_angle,
                    )
                    continue
                elif key == '1':
                    self.send_angles([0, 0, 0, 0, 0, 0])
                    time.sleep(2)
                    self.record_coords = list(self.curr_coords)
                    self.ready_for_coords = False
                    rospy.logwarn("Returned to zero. Press '2' to enable coordinate control.")
                elif key == '2':
                    lower, upper = self.firmware_j5_limits()
                    if not lower <= self.home_pose[4] <= upper:
                        rospy.logerr(
                            "Home pose firmware J5 %.2f° outside limits [%.2f, %.2f]; "
                            "not sending. Re-save with key 3 after fixing pose.",
                            self.home_pose[4], lower, upper,
                        )
                        continue
                    rospy.loginfo(
                        "Home pose command (firmware): %s (J5=%.2f°)",
                        self.format_angles(self.home_pose),
                        self.home_pose[4],
                    )
                    self.send_firmware_angles(self.home_pose)
                    time.sleep(3)
                    self.record_coords = list(self.curr_coords)
                    self.ready_for_coords = True
                    rospy.loginfo(
                        "Home pose command sent. Coordinate control enabled."
                    )
                elif key == '3':
                    # angles_real is logical; key 2 sends firmware_angles_goal.
                    logical = self.format_angles(self.curr_angles)
                    self.home_pose = self.logical_to_firmware_angles(logical)
                    rospy.loginfo(
                        "Updated home pose from logical %s -> firmware %s",
                        logical,
                        self.home_pose,
                    )

                elif key in 'wWsSaAdDzZxXuUiIjJkKoOlL':
                    if not self.ready_for_coords:
                        rospy.logwarn("Coordinate control disabled. Please press '2' first.")
                        continue
                    if key in 'wW':
                        self.record_coords[0] += self.change_len
                    elif key in 'sS':
                        self.record_coords[0] -= self.change_len
                    elif key in 'aA':
                        self.record_coords[1] += self.change_len
                    elif key in 'dD':
                        self.record_coords[1] -= self.change_len
                    elif key in 'zZ':
                        self.record_coords[2] -= self.change_len
                    elif key in 'xX':
                        self.record_coords[2] += self.change_len
                    elif key in 'uU':
                        self.record_coords[3] += self.change_angle
                    elif key in 'jJ':
                        self.record_coords[3] -= self.change_angle
                    elif key in 'iI':
                        self.record_coords[4] += self.change_angle
                    elif key in 'kK':
                        self.record_coords[4] -= self.change_angle
                    elif key in 'oO':
                        self.record_coords[5] += self.change_angle
                    elif key in 'lL':
                        self.record_coords[5] -= self.change_angle

                elif key in 'gGhH':
                    if not self.gripper_enabled or self.gripper_pub is None:
                        rospy.logwarn_throttle(
                            2.0,
                            "Gripper keys ignored: no gripper_controller in robot_description",
                        )
                        continue
                    if key in 'gG':
                        self.gripper_pub.publish(MycobotGripperStatus(Status=True))
                    else:
                        self.gripper_pub.publish(MycobotGripperStatus(Status=False))
                    continue
                else:
                    continue

                if self.ready_for_coords:
                    for val, axis in zip(self.record_coords, COORD_AXES):
                        min_v, max_v = self.coord_limits[axis]
                        if not (min_v <= val <= max_v):
                            rospy.logwarn(
                                "{} Out of range: {} not in [{}, {}]".format(
                                    axis, val, min_v, max_v
                                )
                            )
                            raise ValueError("Out of range of motion")
                    self.send_coords()

            except Exception as e:
                rospy.logwarn("Execution failed: {}".format(e))
                continue


if __name__ == '__main__':
    try:
        teleop = MycobotTeleopTopic()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
