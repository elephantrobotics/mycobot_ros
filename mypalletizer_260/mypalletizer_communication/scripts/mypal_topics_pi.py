#!/usr/bin/env python2
# -*- coding:utf-8 -*-
import time
import os
import sys
import signal
import threading
import rospy
from mypalletizer_communication.msg import(
    MypalAngles,
    MypalCoords,
    MypalSetAngles,
    MypalSetCoords,
    MypalGripperStatus,
    MypalPumpStatus,
)
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
    from pymycobot.mypalletizer260 import MyPalletizer260


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
        the child thread.创建一个返回的子线程。 父线程等待 KeyboardInterrupt 
        然后杀死子线程。
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


class MypalTopics(object):
    def __init__(self):
        super(MypalTopics, self).__init__()

        rospy.init_node("mypal_topics_pi")
        rospy.loginfo("start ...")
        # problem
        port = rospy.get_param("~port", os.popen("ls /dev/ttyAMA*").readline()[:-1])
        baud = rospy.get_param("~baud", 1000000)
        rospy.loginfo("%s,%s" % (port, baud))
        # self.mc = MyCobotSocket(port, baud) # port
        self.mc = MyPalletizer260(port, baud)
        # self.mc.connect()   #pi

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
        pub = rospy.Publisher("mypal/angles_real", MypalAngles, queue_size=5)
        ma = MypalAngles()
        while not rospy.is_shutdown():
            self.lock.acquire()
            angles = self.mc.get_angles()
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
        pub = rospy.Publisher("mypal/coords_real", MypalCoords, queue_size=5)
        ma = MypalCoords()

        while not rospy.is_shutdown():
            self.lock.acquire()
            coords = self.mc.get_coords()
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
                data.joint_5,
                # data.joint_6,
            ]
            sp = int(data.speed)
            self.mc.send_angles(angles, sp)

        sub = rospy.Subscriber(
            "mypal/angles_goal", MypalSetAngles, callback=callback
        )
        rospy.spin()

    def sub_set_coords(self):
        def callback(data):
            # angles = [data.x, data.y, data.z, data.rx, data.ry, data.rz]
            angles = [data.x, data.y, data.z, data.rx]
            sp = int(data.speed)
            model = int(data.model)
            self.mc.send_coords(angles, sp, model)

        sub = rospy.Subscriber(
            "mypal/coords_goal", MypalSetCoords, callback=callback
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
            "mypal/gripper_status", MypalGripperStatus, callback=callback
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
            "mypal/pump_status", MypalPumpStatus, callback=callback
        )
        rospy.spin()

if __name__ == "__main__":
    Watcher()
    mc_topics = MypalTopics()
    mc_topics.start()
    # while True:
    #     mc_topics.pub_real_coords()
    # mc_topics.sub_set_angles()
    pass
