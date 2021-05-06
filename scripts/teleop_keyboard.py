#!/usr/bin/env python
from __future__ import print_function
from myCobotROS.srv import (
    GetCoords, SetCoords, GetAngles, SetAngles, GripperStatus)
import rospy
import sys
import select
import termios
import tty
import copy
import time

import roslib
roslib.load_manifest('mycobot_ros')


msg = """
Mycobot Teleop Keyboard Controller
---------------------------
Movimg options(control coord [x,y,z,rx,ry,rz]):
              w(x+)

    a(y-)     s(x-)     d(y+)

u(rx+)   i(ry+)   o(rz+)
j(rx-)   k(ry-)   l(rz-)

Gripper control:
    g - open
    h - close

Other:
    1 - Go to init pose
    2 -
    3 -
    q - Quit
"""


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tchange size %s " % (speed, turn)


def teleop_keyboard():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')

    model = 0
    speed = rospy.get_param("~speed", 50)
    change_size = rospy.get_param("~change_size", 10)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    rospy.wait_for_service('get_joint_angles')
    rospy.wait_for_service('set_joint_angles')
    rospy.wait_for_service('get_joint_coords')
    rospy.wait_for_service('set_joint_coords')
    rospy.wait_for_service('switch_gripper_status')
    try:
        get_coords = rospy.ServiceProxy('get_joint_coords', GetCoords)
        set_coords = rospy.ServiceProxy('set_joint_coords', SetCoords)
        get_angles = rospy.ServiceProxy('get_joint_angles', GetAngles)
        set_angles = rospy.ServiceProxy('set_joint_angles', SetAngles)
        switch_gripper = rospy.ServiceProxy(
            'switch_gripper_status', GripperStatus)
    except:
        print('start error ...')
        exit(1)

    init_pose = [0, 0, 0, 0, 0, 0, speed]
    home_pose = [0, 0, 0, 0, 0, 0, speed]

    # rsp = set_angles(*init_pose)

    while True:
        res = get_coords()
        if res.x > 5:
            break
        time.sleep(.1)

    record_coords = [
        res.x, res.y, res.z, res.rx, res.ry, res.rz, speed, model
    ]
    print(record_coords)

    try:
        print(msg)
        print(vels(speed, change_size))
        while(1):
            try:
                key = getKey(key_timeout)
                if key == 'q':
                    break
                elif key in ['w', 'W']:
                    record_coords[0] += change_size
                    set_coords(*record_coords)
                elif key in ['s', 'S']:
                    record_coords[0] -= change_size
                    set_coords(*record_coords)
                elif key in ['a', 'A']:
                    record_coords[1] -= change_size
                    set_coords(*record_coords)
                elif key in ['d', 'D']:
                    record_coords[1] += change_size
                    set_coords(*record_coords)
                elif key in ['z', 'Z']:
                    record_coords[2] -= change_size
                    set_coords(*record_coords)
                elif key in ['x', 'X']:
                    record_coords[2] += change_size
                    set_coords(*record_coords)
                elif key in ['u', 'U']:
                    record_coords[3] += change_size
                    set_coords(*record_coords)
                elif key in ['j', 'J']:
                    record_coords[3] -= change_size
                    set_coords(*record_coords)
                elif key in ['i', 'I']:
                    record_coords[4] += change_size
                    set_coords(*record_coords)
                elif key in ['k', 'K']:
                    record_coords[4] -= change_size
                    set_coords(*record_coords)
                elif key in ['o', 'O']:
                    record_coords[5] += change_size
                    set_coords(*record_coords)
                elif key in ['l', 'L']:
                    record_coords[5] -= change_size
                    set_coords(*record_coords)
                elif key in ['g', 'G']:
                    switch_gripper(True)
                elif key in ['h', 'H']:
                    switch_gripper(False)
                elif key == '1':
                    rsp = set_angles(*init_pose)
                elif key in '2':
                    rsp = set_angles(*home_pose)
                elif key in '3':
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
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    try:
        teleop_keyboard()
    except rospy.ROSInterruptException:
        pass
