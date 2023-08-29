#!/usr/bin/env python3
from __future__ import print_function
from cobotx_a450_communication.srv import GetCoords, SetCoords, GetAngles, SetAngles, GripperStatus
import rospy
import sys
import select
import termios
import tty
import time

import roslib

# Terminal output prompt information. 终端输出提示信息
msg = """\
CobotX Teleop Keyboard Controller
---------------------------
Movimg options(control coordinations [x,y,z,rx,ry,rz]):
              w(x+)

    a(y-)     s(x-)     d(y+)

    z(z-) x(z+)

u(rx+)   i(ry+)   o(rz+)
j(rx-)   k(ry-)   l(rz-)

Gripper control:
    g - open
    h - close

Other:
    1 - Go to init pose
    2 - Go to home pose
    3 - Resave home pose
    q - Quit
"""


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


def teleop_keyboard():
    rospy.init_node("teleop_keyboard")

    model = 1
    speed = rospy.get_param("~speed", 50)
    change_percent = rospy.get_param("~change_percent", 5)

    change_angle = 180 * change_percent / 100
    change_len = 250 * change_percent / 100

    rospy.wait_for_service("get_joint_angles")
    rospy.wait_for_service("set_joint_angles")
    rospy.wait_for_service("get_joint_coords")
    rospy.wait_for_service("set_joint_coords")
    rospy.wait_for_service("switch_gripper_status")
    print("service ready.")
    try:
        get_coords = rospy.ServiceProxy("get_joint_coords", GetCoords)
        set_coords = rospy.ServiceProxy("set_joint_coords", SetCoords)
        get_angles = rospy.ServiceProxy("get_joint_angles", GetAngles)
        set_angles = rospy.ServiceProxy("set_joint_angles", SetAngles)
        switch_gripper = rospy.ServiceProxy(
            "switch_gripper_status", GripperStatus)
    except:
        print("start error ...")
        exit(1)

    init_pose = [0, 0, 0, 0, 0, 0, 0, speed]
    home_pose = [0, 0, 0, -90, 0, 90, 0, speed]

    rsp = set_angles(*home_pose)

    while True:
        res = get_coords()
        if res.x > 1:
            break
        time.sleep(0.1)

    record_coords = [res.x, res.y, res.z, res.rx, res.ry, res.rz, speed, model]
    print('init_coords:', record_coords)

    try:
        print(msg)
        print(vels(speed, change_percent))
        # Keyboard keys call different motion functions. 键盘按键调用不同的运动功能
        while 1:
            try:
                # print("\r current coords: %s" % record_coords, end="")
                with Raw(sys.stdin):
                    key = sys.stdin.read(1)
                if key == "q":
                    break
                elif key in ["w", "W"]:
                    record_coords[0] += change_len
                    set_coords(*record_coords)
                elif key in ["s", "S"]:
                    record_coords[0] -= change_len
                    set_coords(*record_coords)
                elif key in ["a", "A"]:
                    record_coords[1] -= change_len
                    set_coords(*record_coords)
                elif key in ["d", "D"]:
                    record_coords[1] += change_len
                    set_coords(*record_coords)
                elif key in ["z", "Z"]:
                    record_coords[2] -= change_len
                    set_coords(*record_coords)
                elif key in ["x", "X"]:
                    record_coords[2] += change_len
                    set_coords(*record_coords)
                elif key in ["u", "U"]:
                    record_coords[3] += change_angle
                    set_coords(*record_coords)
                elif key in ["j", "J"]:
                    record_coords[3] -= change_angle
                    set_coords(*record_coords)
                elif key in ["i", "I"]:
                    record_coords[4] += change_angle
                    set_coords(*record_coords)
                elif key in ["k", "K"]:
                    record_coords[4] -= change_angle
                    set_coords(*record_coords)
                elif key in ["o", "O"]:
                    record_coords[5] += change_angle
                    set_coords(*record_coords)
                elif key in ["l", "L"]:
                    record_coords[5] -= change_angle
                    set_coords(*record_coords)
                elif key in ["g", "G"]:
                    switch_gripper(True)
                elif key in ["h", "H"]:
                    switch_gripper(False)
                elif key == "1":
                    rsp = set_angles(*init_pose)
                elif key in "2":
                    rsp = set_angles(*home_pose)
                    time.sleep(3)
                    res = get_coords()
                    time.sleep(0.1)
                    record_coords = [res.x, res.y, res.z, res.rx, res.ry, res.rz, speed, model]
                    print('home_coords:', record_coords)
                elif key in "3":
                    rep = get_angles()
                    home_pose[0] = rep.joint_1
                    home_pose[1] = rep.joint_2
                    home_pose[2] = rep.joint_3
                    home_pose[3] = rep.joint_4
                    home_pose[5] = rep.joint_5
                else:
                    continue

            except Exception as e:
                # print(e)
                continue

    except Exception as e:
        print(e)


if __name__ == "__main__":
    try:
        teleop_keyboard()
    except rospy.ROSInterruptException:
        pass
