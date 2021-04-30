#!/usr/bin/env python2
import rospy
from myCobotROS.srv import *

from pymycobot.mycobot import MyCobot

mc = None


def mycobot_services():
    global mc
    rospy.init_node('mycobot_services')
    rospy.loginfo('start ...')
    port = rospy.get_param('~port')
    baud = rospy.get_param('~baud')
    rospy.loginfo("%s,%s" % (port, baud))
    mc = MyCobot(port, baud)

    rospy.Service('set_joint_angles', SetAngles, set_angles)
    rospy.Service('get_joint_angles', GetAngles, get_angles)
    rospy.Service('set_joint_coords', SetCoords, set_coords)
    rospy.Service('get_joint_coords', GetCoords, get_coords)
    rospy.Service('switch_gripper_status', GripperStatus, switch_status)
    rospy.loginfo('ready')
    rospy.spin()


def set_angles(req):
    angles = [
        req.joint_1,
        req.joint_2,
        req.joint_3,
        req.joint_4,
        req.joint_5,
        req.joint_6,
    ]
    sp = req.speed

    if mc:
        mc.send_angles(angles, sp)


def get_angles(req):
    if mc:
        angles = mc.get_angles()
        print(angles)
        return GetAnglesResponse(*angles)


def set_coords(req):
    coords = [
        req.x,
        req.y,
        req.z,
        req.rx,
        req.ry,
        req.rz,
    ]
    sp = req.speed
    mod = req.model

    if mc:
        mc.send_coords(coords, sp, mod)


def get_coords(req):
    if mc:
        coords = mc.get_coords()
        return GetCoordsResponse(*coords)


def switch_status(req):
    if mc:
        if req.Status:
            print(1)
            mc.set_gripper_state(0, 80)
        else:
            print(2)
            mc.set_gripper_state(1, 80)


if __name__ == '__main__':
    # print(MyCobot.__dict__)
    mycobot_services()
