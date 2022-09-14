#!/usr/bin/env python2
# -*- coding:utf-8 -*-
import time
import os
import sys
import signal
import threading

import rospy

from mycobot_communication.msg import (
    MycobotAngles,
    MycobotCoords,
    MycobotSetAngles,
    MycobotSetCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
)


# from pymycobot import MyCobot

from pymycobot import MyCobotSocket  # pi


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
            # I put the capital B in KeyBoardInterrupt so I can# 
            # 我把大写的 B 放在 KeyBoardInterrupt 中，这样我就可以了
            # tell when the Watcher gets the SIGINT，告诉 Watcher 何时收到 SIGINT
            print("KeyBoardInterrupt")
            self.kill()
        sys.exit()

    def kill(self):
        try:
            os.kill(self.child, signal.SIGKILL)
        except OSError:
            pass


class MycobotTopics(object):
    def __init__(self):
        super(MycobotTopics, self).__init__()

        rospy.init_node("mycobot_topics")
        rospy.loginfo("start ...")
        # Select connected device，选择连接设备
        port = rospy.get_param("~port", "/dev/ttyAMA0")
        baud = rospy.get_param("~baud", 1000000)
        rospy.loginfo("%s,%s" % (port, baud))
        # self.mc = MyCobot(port,baud)
        self.mc = MyCobotSocket(port, baud) # port
        self.mc.connect()   


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
        pub = rospy.Publisher("mycobot/angles_real", MycobotAngles, queue_size=5)
        ma = MycobotAngles()
        while not rospy.is_shutdown():
            self.lock.acquire()
            angles = self.mc.get_angles()
            self.lock.release()
            if angles:
                ma.joint_1 = angles[0]
                ma.joint_2 = angles[1]
                ma.joint_3 = angles[2]
                ma.joint_4 = angles[3]
                ma.joint_5 = angles[4]
                ma.joint_6 = angles[5]
                pub.publish(ma)
            time.sleep(0.25)

    def pub_real_coords(self):
        """publish real coordinates"""
        """发布真实坐标"""
        pub = rospy.Publisher("mycobot/coords_real", MycobotCoords, queue_size=5)
        ma = MycobotCoords()

        while not rospy.is_shutdown():
            self.lock.acquire()
            coords = self.mc.get_coords()
            self.lock.release()
            if coords:
                ma.x = coords[0]
                ma.y = coords[1]
                ma.z = coords[2]
                ma.rx = coords[3]
                ma.ry = coords[4]
                ma.rz = coords[5]
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
        """Subscribe to coordinates"""
        """订阅坐标"""
        def callback(data):
            angles = [data.x, data.y, data.z, data.rx, data.ry, data.rz]
            sp = int(data.speed)
            model = int(data.model)
            self.mc.send_coords(angles, sp, model)

        sub = rospy.Subscriber(
            "mycobot/coords_goal", MycobotSetCoords, callback=callback
        )
        rospy.spin()

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
                self.mc.set_basic_output(data.Pin1, 0)
                self.mc.set_basic_output(data.Pin2, 0)
            else:
                self.mc.set_basic_output(data.Pin1, 1)
                self.mc.set_basic_output(data.Pin2, 1)

        sub = rospy.Subscriber(
            "mycobot/pump_status", MycobotPumpStatus, callback=callback
        )
        rospy.spin()

if __name__ == "__main__":
    Watcher()
    mc_topics = MycobotTopics()
    mc_topics.start()
    # while True:
    #     mc_topics.pub_real_coords()
    # mc_topics.sub_set_angles()
    pass
