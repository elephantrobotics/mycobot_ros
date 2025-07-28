#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
import os
import fcntl
from mycobot_communication.srv import *

import pymycobot
from packaging import version

# min low version require
MAX_REQUIRE_VERSION = '3.5.3'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot import MyCobot

mc = None

# Avoid serial port conflicts and need to be locked
def acquire(lock_file):
    open_mode = os.O_RDWR | os.O_CREAT | os.O_TRUNC
    fd = os.open(lock_file, open_mode)

    pid = os.getpid()
    lock_file_fd = None
    
    timeout = 50.0
    start_time = current_time = time.time()
    while current_time < start_time + timeout:
        try:
            # The LOCK_EX means that only one process can hold the lock
            # The LOCK_NB means that the fcntl.flock() is not blocking
            # and we are able to implement termination of while loop,
            # when timeout is reached.
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (IOError, OSError):
            pass
        else:
            lock_file_fd = fd
            break
        print('pid waiting for lock:%d'% pid)
        time.sleep(1.0)
        current_time = time.time()
    if lock_file_fd is None:
        os.close(fd)
    return lock_file_fd


def release(lock_file_fd):
    # Do not remove the lockfile:
    fcntl.flock(lock_file_fd, fcntl.LOCK_UN)
    os.close(lock_file_fd)
    return None


def create_handle():
    global mc
    rospy.init_node("mycobot_services")
    rospy.loginfo("start ...")
    port = rospy.get_param("~port")
    baud = rospy.get_param("~baud")
    rospy.loginfo("%s,%s" % (port, baud))
    mc = MyCobot(port, baud)


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
    """set angles，设置角度"""
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
        lock = acquire("/tmp/mycobot_lock")
        mc.send_angles(angles, sp)
        release(lock)

    return SetAnglesResponse(True)


def get_angles(req):
    """get angles,获取角度"""
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        angles = mc.get_angles()
        release(lock)
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
        lock = acquire("/tmp/mycobot_lock")
        mc.send_coords(coords, sp, mod)
        release(lock)

    return SetCoordsResponse(True)


def get_coords(req):
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        coords = mc.get_coords()
        release(lock)
        return GetCoordsResponse(*coords)


def switch_status(req):
    """Gripper switch status"""
    """夹爪开关状态"""
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        if req.Status:
            mc.set_gripper_state(0, 80)
        else:
            mc.set_gripper_state(1, 80)
        release(lock)

    return GripperStatusResponse(True)


def toggle_pump(req):
    if mc:
        lock = acquire("/tmp/mycobot_lock")
        if req.Status:
            mc.set_basic_output(req.Pin1, 0)
            mc.set_basic_output(req.Pin2, 0)
        else:
            mc.set_basic_output(req.Pin1, 1)
            mc.set_basic_output(req.Pin2, 1)
        release(lock)
    return PumpStatusResponse(True)


robot_msg = """
MyCobot Status
--------------------------------
Joint Limit:
    joint 1: -170 ~ +170
    joint 2: -170 ~ +170
    joint 3: -170 ~ +170
    joint 4: -170 ~ +170
    joint 5: -170 ~ +170
    joint 6: -180 ~ +180

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

    if mc:
        lock = acquire("/tmp/mycobot_lock")
        cn = mc.is_controller_connected()
        release(lock)
        if cn == 1:
            connect_status = True
        time.sleep(0.1)
        lock = acquire("/tmp/mycobot_lock")
        si = mc.is_all_servo_enable()
        release(lock)
        if si == 1:
            servo_infomation = "all connected"

    print(
        robot_msg % (connect_status, servo_infomation,
                     servo_temperature, atom_version)
    )


if __name__ == "__main__":
    # print(MyCobot.__dict__)
    create_handle()
    output_robot_message()
    create_services()
