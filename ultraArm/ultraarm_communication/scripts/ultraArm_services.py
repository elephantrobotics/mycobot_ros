#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
from ultraarm_communication.srv import *
from pymycobot.ultraArm import ultraArm

ua = None


def create_handle():
    global ua
    rospy.init_node("ultraArm_services")
    rospy.loginfo("start ...")
    port = rospy.get_param("~port")
    baud = rospy.get_param("~baud")
    rospy.loginfo("%s,%s" % (port, baud))
    ua = ultraArm(port, baud)
    
    # Power on the robotic arm
    ua.power_on()
    # calibrate the zero position of the robot arm
    ua.go_zero()
    # time.sleep(3)


def create_services():
    rospy.Service("set_joint_angles", SetAngles, set_angles)
    rospy.Service("get_joint_angles", GetAngles, get_angles)
    rospy.Service("set_joint_coords", SetCoords, set_coords)
    rospy.Service("get_joint_coords", GetCoords, get_coords)
    rospy.Service("switch_gripper_status", GripperStatus, switch_status)
    rospy.Service("switch_pump_status", PumpStatus, toggle_pump)
    rospy.loginfo("ready")
    rospy.spin()


def set_angles(req):
    """set angles"""
    angles = [
        req.joint_1,
        req.joint_2,
        req.joint_3,
    ]
    sp = req.speed
    if ua:
        ua.set_angles(angles, sp)

    return SetAnglesResponse(True)


def get_angles(req):
    count = 0
    while count < 10:
        if ua:
            angles = ua.get_angles_info()
            if angles != None:
                return GetAnglesResponse(*angles)
            count += 1
            continue
    else:
        return GetAnglesResponse(0.0, 0.0, 0.0)


def set_coords(req):
    coords = [
        req.x,
        req.y,
        req.z,
    ]
    sp = req.speed
    if ua:
        ua.set_coords(coords, sp)

    return SetCoordsResponse(True)


def get_coords(req):
    count = 0
    while count < 10:
        if ua:
            coords = ua.get_coords_info()
            # coords = [176.0, 0.0, 120.0]
            if coords != None:
                return GetCoordsResponse(*coords)
            count += 1
            continue
    else:
        return GetCoordsResponse(176.0, 0.0, 120.0)


def switch_status(req):
    """Gripper switch,夹爪开关"""
    if ua:
        if req.Status:
            ua.set_gripper_state(0)
        else:
            ua.set_gripper_state(1)

    return GripperStatusResponse(True)


def toggle_pump(req):
    if ua:
        if req.Status:
            ua.set_gpio_state(1)
        else:
            ua.set_gpio_state(0)

    return PumpStatusResponse(True)


robot_msg = """
ultraArm Status
--------------------------------
Joint Limit:
    joint 1: -170 ~ +170
    joint 2: 0 ~ 90
    joint 3: 0 ~ 75

Coords Limit:
    x: 0 ~ 270 
    y: 0 ~ 270 
    z: 0 ~ 125 
    
Connect Status: %s

Servo Infomation: %s

Servo Temperature: %s
"""


def output_robot_message():
    connect_status = False
    servo_infomation = "unknown"
    servo_temperature = "unknown"

    if ua:
        connect_status = True
        servo_infomation = "all connected"

    print(
        robot_msg % (connect_status, servo_infomation,
                     servo_temperature)
    )


if __name__ == "__main__":
    # print(MyCobot.__dict__)
    create_handle()
    output_robot_message()
    create_services()
