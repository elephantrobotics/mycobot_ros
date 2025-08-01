#!/usr/bin/env python2
# -*- coding:utf-8 -*-
import time
import os
import sys
import signal
import threading
import traceback

import rospy

from mycobot_communication.msg import (
    MycobotAngles,
    MycobotCoords,
    MycobotSetAngles,
    MycobotSetCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
    MycobotSetEndType,
    MycobotSetFreshMode,
    MycobotSetToolReference,
    MycobotSetVisionMode,
    MycobotGetGripperValue,
)
from std_msgs.msg import UInt8
import pymycobot
from packaging import version
# min low version require
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280


class Watcher:
    """this class solves two problems with multithreaded
    programs in Python, (1) a signal might be delivered
    to any thread (which is just a malfeature) and (2) if
    the thread that gets the signal is waiting, the signal
    is ignored (which is a bug).

    The watcher is a concurrent process (not thread) that
    waits for a signal and the process that contains the
    threads.  See Appendix A of The Little Book of Semaphores.
    http://greenteapress.com/semaphores/

    I have only tested this on Linux.  I would expect it to
    work on the Macintosh and not work on Windows.
    """

    def __init__(self):
        """Creates a child thread, which returns.  The parent
        thread waits for a KeyboardInterrupt and then kills
        the child thread.
        """
        self.child = os.fork()
        if self.child == 0:
            return
        else:
            self.watch()

    def watch(self):
        try:
            os.wait()
        except KeyboardInterrupt:
            # I put the capital B in KeyBoardInterrupt so I can
            # tell when the Watcher gets the SIGINT
            print("KeyBoardInterrupt")
            self.kill()
        sys.exit()

    def kill(self):
        try:
            os.kill(self.child, signal.SIGKILL)
        except OSError:
            pass
robot_msg = """
MyCobot Status
--------------------------------
Joint Limit:
    joint 1: -168 ~ +168
    joint 2: -135 ~ +135
    joint 3: -150 ~ +150
    joint 4: -145 ~ +145
    joint 5: -165 ~ +165
    joint 6: -180 ~ +180

Connect Status: %s

Servo Infomation: %s

Servo Temperature: %s

Atom Version: %s
"""

class MycobotTopics(object):
    def __init__(self):
        super(MycobotTopics, self).__init__()
        rospy.loginfo("start ...")
        rospy.init_node("mycobot_topics")
        port = rospy.get_param("~port", os.popen("ls /dev/ttyUSB*").readline()[:-1])
        if not port:
            port = rospy.get_param("~port", os.popen("ls /dev/ttyACM*").readline()[:-1])
        baud = rospy.get_param("~baud", 115200)
        rospy.loginfo("%s,%s" % (port, baud))
        self.mc = MyCobot280(port, baud)
        self.lock = threading.Lock()
        self.mc.set_vision_mode(1)
        self.output_robot_message()

    def start(self):
        pa = threading.Thread(target=self.pub_real_angles)
        pb = threading.Thread(target=self.pub_real_coords)
        sa = threading.Thread(target=self.sub_set_angles)
        sb = threading.Thread(target=self.sub_set_coords)
        sg = threading.Thread(target=self.sub_gripper_status)
        sp = threading.Thread(target=self.sub_pump_status)
        
        sfm = threading.Thread(target=self.sub_fresh_mode_status)
        set = threading.Thread(target=self.sub_end_type_status)
        str = threading.Thread(target=self.sub_set_tool_reference)
        svm = threading.Thread(target=self.sub_vision_mode_status)
        
        sgv = threading.Thread(target=self.sub_real_gripper_value)

        pa.setDaemon(True)
        pa.start()
        pb.setDaemon(True)
        pb.start()
        sa.setDaemon(True)
        sa.start()
        sb.setDaemon(True)
        sb.start()
        sg.setDaemon(True)
        sg.start()
        sp.setDaemon(True)
        sp.start()
        
        sfm.setDaemon(True)
        sfm.start
        set.setDaemon(True)
        set.start()
        str.setDaemon(True)
        str.start()
        svm.setDaemon(True)
        svm.start
        
        sgv.setDaemon(True)
        sgv.start()

        pa.join()
        pb.join()
        sa.join()
        sb.join()
        sg.join()
        sp.join()
        
        sfm.join()
        set.join()
        str.join()
        svm.join()
        sgv.join()

    def pub_real_angles(self):
        """Publish real angle"""
        """发布真实角度"""
        pub = rospy.Publisher("mycobot/angles_real",
                              MycobotAngles, queue_size=5)
        ma = MycobotAngles()
        while not rospy.is_shutdown():
            with self.lock:
                try:
                    angles = self.mc.get_angles()
                    if isinstance(angles, list) and len(angles) == 6 and all(c != -1 for c in angles):
                        ma.joint_1 = angles[0]
                        ma.joint_2 = angles[1]
                        ma.joint_3 = angles[2]
                        ma.joint_4 = angles[3]
                        ma.joint_5 = angles[4]
                        ma.joint_6 = angles[5]
                        pub.publish(ma)
                    else:
                        rospy.logwarn("None or -1")
                except Exception as e:
                    e = traceback.format_exc()
                    rospy.logerr(f"SerialException: {e}")
            time.sleep(0.25)

    def pub_real_coords(self):
        """publish real coordinates"""
        """发布真实坐标"""
        pub = rospy.Publisher("mycobot/coords_real",
                              MycobotCoords, queue_size=5)
        ma = MycobotCoords()

        while not rospy.is_shutdown():
            with self.lock:
                try:
                    coords = self.mc.get_coords()
                    if isinstance(coords, list) and len(coords) == 6 and all(c != -1 for c in coords):
                        ma.x = coords[0]
                        ma.y = coords[1]
                        ma.z = coords[2]
                        ma.rx = coords[3]
                        ma.ry = coords[4]
                        ma.rz = coords[5]
                        pub.publish(ma)
                    else:
                        rospy.logwarn("None or -1")
                except Exception as e:
                    e = traceback.format_exc()
                    rospy.logerr(f"SerialException: {e}")
            time.sleep(0.25)

    def sub_set_angles(self):
        """subscription angles"""
        """订阅角度"""
        def callback(data):
            angles = [
                data.joint_1,
                data.joint_2,
                data.joint_3,
                data.joint_4,
                data.joint_5,
                data.joint_6,
            ]
            sp = int(data.speed)
            self.mc.send_angles(angles, sp)

        sub = rospy.Subscriber(
            "mycobot/angles_goal", MycobotSetAngles, callback=callback
        )
        rospy.spin()

    def sub_set_coords(self):
        def callback(data):
            angles = [data.x, data.y, data.z, data.rx, data.ry, data.rz]
            sp = int(data.speed)
            model = int(data.model)
            self.mc.send_coords(angles, sp, model)

        sub = rospy.Subscriber(
            "mycobot/coords_goal", MycobotSetCoords, callback=callback
        )
        rospy.spin()

    def sub_real_gripper_value(self):
        """Get Gripper Value"""
        pub = rospy.Publisher("mycobot/gripper_angle_real",
                              MycobotGetGripperValue, queue_size=5)
        ma = MycobotGetGripperValue()
        while not rospy.is_shutdown():
            with self.lock:
                try:
                    gripper_value = self.mc.get_gripper_value()
                    if gripper_value:
                        ma.gripper_angle = gripper_value
                        pub.publish(ma)
                except Exception as e:
                    rospy.logerr(f"SerialException: {e}")
            time.sleep(0.25)
            
    def sub_gripper_status(self):
        """Subscribe to Gripper Status"""
        """订阅夹爪状态"""
        def callback(data):
            if data.Status:
                self.mc.set_gripper_state(0, 80)
            else:
                self.mc.set_gripper_state(1, 80)

        sub = rospy.Subscriber(
            "mycobot/gripper_status", MycobotGripperStatus, callback=callback
        )
        rospy.spin()

    def sub_pump_status(self):
        def callback(data):
            if data.Status:
                self.mc.set_basic_output(data.Pin2, 0)
                time.sleep(0.05)
            else:
                self.mc.set_basic_output(data.Pin2, 1)
                time.sleep(0.05)
                self.mc.set_basic_output(data.Pin1, 0)
                time.sleep(0.05)
                self.mc.set_basic_output(data.PIn1, 1)
                time.sleep(0.05)

        sub = rospy.Subscriber(
            "mycobot/pump_status", MycobotPumpStatus, callback=callback
        )
        rospy.spin()
        
    def sub_fresh_mode_status(self):
        """Subscribe to fresh mode Status"""
        """订阅运动模式状态"""
        def callback(data):
            if data.Status==1:
                self.mc.set_fresh_mode(1)
            else:
                self.mc.set_fresh_mode(0)

        sub = rospy.Subscriber(
            "mycobot/fresh_mode_status", MycobotSetFreshMode, callback=callback
        )
        rospy.spin()
    
    def sub_vision_mode_status(self):
        """Subscribe to vision mode Status"""
        """订阅运动模式状态"""
        def callback(data):
            if data.Status==1:
                self.mc.set_vision_mode(1)
            elif data.Status==2:
                self.mc.stop()
            else:
                self.mc.set_vision_mode(0)

        sub = rospy.Subscriber(
            "mycobot/vision_mode_status", MycobotSetVisionMode, callback=callback
        )
        rospy.spin()
        
    def sub_end_type_status(self):
        """Subscribe to end type Status"""
        """订阅末端类型状态"""
        def callback(data):
            if data.Status==1:
                self.mc.set_end_type(1)
            else:
                self.mc.set_end_type(0)

        sub = rospy.Subscriber(
            "mycobot/end_type_status", MycobotSetEndType, callback=callback
        )
        rospy.spin()
        
    def sub_set_tool_reference(self):
        def callback(data):
            coords = [data.x, data.y, data.z, data.rx, data.ry, data.rz]
            self.mc.set_tool_reference(coords)

        sub = rospy.Subscriber(
            "mycobot/tool_reference_goal", MycobotSetToolReference, callback=callback
        )
        rospy.spin()
        

    def output_robot_message(self):
        connect_status = False
        servo_infomation = "unknown"
        servo_temperature = "unknown"
        atom_version = "unknown"

        if self.mc:
            cn = self.mc.is_controller_connected()
            if cn == 1:
                connect_status = True
            time.sleep(0.1)
            si = self.mc.is_all_servo_enable()
            if si == 1:
                servo_infomation = "all connected"
            version = self.mc.get_system_version()
            if version:
                atom_version = version

        print(
            robot_msg % (connect_status, servo_infomation,
                        servo_temperature, atom_version)
        )


if __name__ == "__main__":
    Watcher()
    mc_topics = MycobotTopics()
    mc_topics.start()
    # while True:
    #     mc_topics.pub_real_coords()
    # mc_topics.sub_set_angles()
    pass
