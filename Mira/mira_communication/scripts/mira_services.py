#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
# from mycobot_communication.srv import *
from mira_communication.srv import *

from pymycobot.mira import Mira

ma = None


def create_handle():
    global ma
    rospy.init_node("mira_services")
    rospy.loginfo("start ...")
    port = rospy.get_param("~port")
    baud = rospy.get_param("~baud")
    rospy.loginfo("%s,%s" % (port, baud))
    ma = Mira(port, baud)
    
    # Power on the robotic arm
    ma.power_on()
    # calibrate the zero position of the robot arm
    ma.go_zero()
    
    time.sleep(5)


def create_services():
    rospy.Service("set_joint_angles", SetAngles, set_angles)
    rospy.Service("get_joint_angles", GetAngles, get_angles)
    rospy.Service("set_joint_coords", SetCoords, set_coords)
    rospy.Service("get_joint_coords", GetCoords, get_coords)
    rospy.Service("switch_gripper_status", GripperStatus, switch_status)
    # rospy.Service("switch_pump_status", PumpStatus, toggle_pump)
    rospy.loginfo("ready")
    rospy.spin()


def set_angles(req):
    angles = [
        req.joint_1,
        req.joint_2,
        req.joint_3,
    ]
    sp = req.speed
    print('mira_services:',angles)
    if ma:
        ma.set_angles(angles[0], angles[1], angles[2], sp)

    return SetAnglesResponse(True)


def get_angles(req):
    if ma:
        angles = ma.get_angles_info()
        # angles = [0.0, 0.0, 0.0, 0.0]
        if angles != None:
            angles_data = angles[:3]
            # print('angles:',angles)
            return GetAnglesResponse(*angles_data)


def set_coords(req):
    coords = [
        req.x,
        req.y,
        req.z,
    ]
    sp = req.speed
    # mod = req.model
    print('mira_services:',coords)
    if ma:
        ma.set_coords(coords[0], coords[1], coords[2], sp)

    return SetCoordsResponse(True)


def get_coords(req):
    if ma:
        # coords = ma.get_coords_info()
        coords = [0.0, 0.0, 0.0, 0.0]
        if coords != None:
            coords_data = coords[:3]
            # print('coords:',coords)
            return GetCoordsResponse(*coords_data)


def switch_status(req):
    """Gripper switch,夹爪开关"""
    if ma:
        if req.Status:
            ma.set_gripper_state(0, 50)
        else:
            ma.set_gripper_state(1, 50)

    return GripperStatusResponse(True)


# def toggle_pump(req):
#     if ma:
#         if req.Status:
#             ma.set_basic_output(req.Pin1, 0)
#             ma.set_basic_output(req.Pin2, 0)
#         else:
#             ma.set_basic_output(req.Pin1, 1)
#             ma.set_basic_output(req.Pin2, 1)

#     return PumpStatusResponse(True)


robot_msg = """
Mira Status
--------------------------------
Joint Limit:
    joint 1: -170 ~ +170
    joint 2: 0 ~ 90
    joint 3: 0 ~ 75

Connect Status: %s

Servo Infomation: %s

Servo Temperature: %s

Atom Version: %s
"""


def output_robot_message():
    connect_status = False
    servo_infomation = "unknown"
    servo_temperature = "unknown"
    atom_version = "unknown"

    if ma:
        # cn = ma.is_controller_connected()
        # if cn == 1:
        connect_status = True
        # time.sleep(0.1)
        # si = ma.is_all_servo_enable()
        # if si == 1:
        servo_infomation = "all connected"

    print(
        robot_msg % (connect_status, servo_infomation,
                     servo_temperature, atom_version)
    )
    time.sleep(2)


if __name__ == "__main__":
    # print(MyCobot.__dict__)
    create_handle()
    output_robot_message()
    create_services()
