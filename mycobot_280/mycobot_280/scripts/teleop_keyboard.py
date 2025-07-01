#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from __future__ import print_function
import sys
import time
import termios
import tty
import math
import rospy

from mycobot_communication.msg import (
    MycobotAngles,
    MycobotCoords,
    MycobotSetAngles,
    MycobotSetCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
)

msg = """\
Mycobot Teleop Keyboard Controller (ROS1 - Topic Version)
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

Gripper:
    g - open    h - close

Pump:
    b - open    m - close

Other:
    1 - Go to init pose
    2 - Go to home pose
    3 - Save current pose as home
    q - Quit
"""

COORD_LIMITS = {
    'x': (-281.45, 281.45),
    'y': (-281.45, 281.45),
    'z': (-70, 420.67),
    'rx': (-180, 180),
    'ry': (-180, 180),
    'rz': (-180, 180)
}

def vels(speed, turn):
    return "currently:\tspeed: %s\tchange percent: %s  " % (speed, turn)

class Raw(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)


class MycobotTeleopTopic:
    def __init__(self):
        rospy.init_node("teleop_keyboard_topic")

        self.speed = rospy.get_param("~speed", 50)
        self.model = 1
        self.change_percent = rospy.get_param("~change_percent", 5)
        self.change_len = 250 * self.change_percent / 100.0
        self.change_angle = 180 * self.change_percent / 100.0

        # 当前坐标和角度
        self.curr_coords = [0] * 6
        self.curr_angles = [0] * 6

        # 保存坐标作为目标值
        self.record_coords = None
        self.home_pose = [0, 8, -127, 40, 0, 0]

        # 订阅器
        rospy.Subscriber("mycobot/coords_real", MycobotCoords, self.coords_callback)
        rospy.Subscriber("mycobot/angles_real", MycobotAngles, self.angles_callback)

        # 发布器
        self.coords_pub = rospy.Publisher("mycobot/coords_goal", MycobotSetCoords, queue_size=1)
        self.angles_pub = rospy.Publisher("mycobot/angles_goal", MycobotSetAngles, queue_size=1)
        self.gripper_pub = rospy.Publisher("mycobot/gripper_status", MycobotGripperStatus, queue_size=1)
        self.pump_pub = rospy.Publisher("mycobot/pump_status", MycobotPumpStatus, queue_size=1)

        rospy.loginfo("Waiting to receive current coordinates...")
        while self.curr_coords == [0] * 6 and not rospy.is_shutdown():
            time.sleep(0.1)

        self.record_coords = list(self.curr_coords)
        print(msg)
        print(vels(self.speed, self.change_percent))
        rospy.loginfo("Current moving step: position %.1f mm, angle attitude %.1f°", self.change_len, self.change_angle)

    def coords_callback(self, msg):
        self.curr_coords = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]

    def angles_callback(self, msg):
        self.curr_angles = [
            msg.joint_1, msg.joint_2, msg.joint_3,
            msg.joint_4, msg.joint_5, msg.joint_6
        ]

    def send_coords(self):
        goal = MycobotSetCoords()
        goal.x, goal.y, goal.z, goal.rx, goal.ry, goal.rz = self.record_coords
        goal.speed = self.speed
        goal.model = self.model
        self.coords_pub.publish(goal)

    def send_angles(self, angles):
        goal = MycobotSetAngles()
        goal.joint_1, goal.joint_2, goal.joint_3, goal.joint_4, goal.joint_5, goal.joint_6 = angles
        goal.speed = self.speed
        self.angles_pub.publish(goal)

    def run(self):
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
                    self.change_percent = min(self.change_percent + 1, 20)  # 最大20%
                    self.change_angle = 180 * self.change_percent / 100.0
                    self.change_len = 250 * self.change_percent / 100.0
                    rospy.loginfo("Increase change_percent to %d%%, move step: %.1f mm, angle step: %.1f°" %
                        (self.change_percent, self.change_len, self.change_angle))
                elif key == '-':
                    self.change_percent = max(self.change_percent - 1, 1)  # 最小1%
                    self.change_angle = 180 * self.change_percent / 100.0
                    self.change_len = 250 * self.change_percent / 100.0
                    rospy.loginfo("Decrease change_percent to %d%%, move step: %.1f mm, angle step: %.1f°" %
                        (self.change_percent, self.change_len, self.change_angle))
                    continue
                elif key in 'wW': self.record_coords[0] += self.change_len
                elif key in 'sS': self.record_coords[0] -= self.change_len
                elif key in 'aA': self.record_coords[1] += self.change_len
                elif key in 'dD': self.record_coords[1] -= self.change_len
                elif key in 'zZ': self.record_coords[2] -= self.change_len
                elif key in 'xX': self.record_coords[2] += self.change_len
                elif key in 'uU': self.record_coords[3] += self.change_angle
                elif key in 'jJ': self.record_coords[3] -= self.change_angle
                elif key in 'iI': self.record_coords[4] += self.change_angle
                elif key in 'kK': self.record_coords[4] -= self.change_angle
                elif key in 'oO': self.record_coords[5] += self.change_angle
                elif key in 'lL': self.record_coords[5] -= self.change_angle
                elif key in 'gG':
                    self.gripper_pub.publish(MycobotGripperStatus(Status=True))
                elif key in 'hH':
                    self.gripper_pub.publish(MycobotGripperStatus(Status=False))
                elif key in 'bB':
                    self.pump_pub.publish(MycobotPumpStatus(Status=True, Pin1=2, Pin2=5))
                elif key in 'mM':
                    self.pump_pub.publish(MycobotPumpStatus(Status=False, Pin1=2, Pin2=5))
                elif key == '1':
                    self.send_angles([0, 0, 0, 0, 0, 0])
                    time.sleep(2)
                    self.record_coords = list(self.curr_coords)
                elif key == '2':
                    self.send_angles(self.home_pose)
                    time.sleep(2)
                    self.record_coords = list(self.curr_coords)
                elif key == '3':
                    self.home_pose = list(self.curr_angles)
                else:
                    continue

                # 范围检查
                for val, axis in zip(self.record_coords, ['x', 'y', 'z', 'rx', 'ry', 'rz']):
                    min_v, max_v = COORD_LIMITS[axis]
                    if not (min_v <= val <= max_v):
                        rospy.logwarn(f"{axis} Out of range: {val} not in [{min_v}, {max_v}]")
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
