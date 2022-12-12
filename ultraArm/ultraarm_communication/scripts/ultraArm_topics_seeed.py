#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import os
import sys
import signal
import threading

import rospy

from ultraarm_communication.msg import (
    ultraArmAngles,
    ultraArmCoords,
    ultraArmSetAngles,
    ultraArmSetCoords,
    ultraArmGripperStatus,
    ultraArmPumpStatus,
)
from pymycobot.ultraArm import ultraArm


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


class ultraArmTopics(object):
    def __init__(self):
        super(ultraArmTopics, self).__init__()

        rospy.init_node("ultraArm_topics")
        rospy.loginfo("start ...")
        # problem
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 115200)
        rospy.loginfo("%s,%s" % (port, baud))
        self.ua = ultraArm(port,baud)
        self.ua.go_zero()
        self.lock = threading.Lock()

    def start(self):
        pa = threading.Thread(target=self.pub_real_angles)
        pb = threading.Thread(target=self.pub_real_coords)
        sa = threading.Thread(target=self.sub_set_angles)
        sb = threading.Thread(target=self.sub_set_coords)
        sg = threading.Thread(target=self.sub_gripper_status)
        sp = threading.Thread(target=self.sub_pump_status)

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

        pa.join()
        pb.join()
        sa.join()
        sb.join()
        sg.join()
        sp.join()

    def pub_real_angles(self):
        """Publish real angle"""
        """发布真实角度"""
        pub = rospy.Publisher("ultraArm/angles_real", ultraArmAngles, queue_size=5)
        ma = ultraArmAngles()
        while not rospy.is_shutdown():
            self.lock.acquire()
            angles = self.ua.get_angles_info()
            self.lock.release()
            if angles:
                ma.joint_1 = angles[0]
                ma.joint_2 = angles[1]
                ma.joint_3 = angles[2]
                ma.joint_4 = angles[3]
                # ma.joint_5 = angles[4]
                # ma.joint_6 = angles[5]
                pub.publish(ma)
            time.sleep(0.25)

    def pub_real_coords(self):
        """publish real coordinates"""
        """发布真实坐标"""
        pub = rospy.Publisher("ultraArm/coords_real", ultraArmCoords, queue_size=5)
        ma = ultraArmCoords()

        while not rospy.is_shutdown():
            self.lock.acquire()
            coords = self.ua.get_coords_info()
            self.lock.release()
            if coords:
                ma.x = coords[0]
                ma.y = coords[1]
                ma.z = coords[2]
                ma.rx = coords[3]
                # ma.ry = coords[4]
                # ma.rz = coords[5]
                pub.publish(ma)
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
                # data.joint_5,
                # data.joint_6,
            ]
            sp = int(data.speed)
            self.ua.set_angles(angles, sp)

        sub = rospy.Subscriber(
            "ultraArm/angles_goal", ultraArmSetAngles, callback=callback
        )
        rospy.spin()

    def sub_set_coords(self):
        def callback(data):
            angles = [data.x, data.y, data.z, data.rx]
            sp = int(data.speed)
            model = int(data.model)
            self.ua.set_coords(angles, sp)

        sub = rospy.Subscriber(
            "ultraArm/coords_goal", ultraArmSetCoords, callback=callback
        )
        rospy.spin()

    def sub_gripper_status(self):
        """Subscribe to Gripper Status"""
        """订阅夹爪状态"""
        def callback(data):
            if data.Status:
                self.ua.set_gripper_state(0, 80)
            else:
                self.ua.set_gripper_state(1, 80)

        sub = rospy.Subscriber(
            "ultraArm/gripper_status", ultraArmGripperStatus, callback=callback
        )
        rospy.spin()

    def sub_pump_status(self):
        def callback(data):
            if data.Status:
                self.ua.set_gpio_state(0)
            else:
                self.ua.set_gpio_state(1)

        sub = rospy.Subscriber(
            "ultraArm/pump_status", ultraArmPumpStatus, callback=callback
        )
        rospy.spin()

if __name__ == "__main__":
    Watcher()
    mc_topics = ultraArmTopics()
    mc_topics.start()
    # while True:
    #     mc_topics.pub_real_coords()
    # mc_topics.sub_set_angles()
    pass
