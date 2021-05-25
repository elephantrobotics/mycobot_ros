#!/usr/bin/env python2
import time
import threading

import rospy

from mycobot_ros.msg import (MycobotAngles, MycobotCoords, MycobotSetAngles, MycobotSetCoords, MycobotGripperStatus, MycobotPumpStatus)
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot


class MycobotTopics(object):

    def __init__(self):
        super(MycobotTopics, self).__init__()

        rospy.init_node('mycobot_topics')
        rospy.loginfo('start ...')
        port = rospy.get_param('~port')
        baud = rospy.get_param('~baud')
        rospy.loginfo("%s,%s" % (port, baud))
        self.mc = MyCobot(port, baud)
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
        pub = rospy.Publisher('mycobot/angles_real', MycobotAngles, queue_size=5)
        ma = MycobotAngles()
        while True:
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
            time.sleep(.25)

    def pub_real_coords(self):
        pub = rospy.Publisher('mycobot/coords_real', MycobotCoords, queue_size=5)
        ma = MycobotCoords()

        while True:
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
            time.sleep(.25)

    def sub_set_angles(self):
        def callback(data):
            angles = [data.joint_1, data.joint_2, data.joint_3, data.joint_4, data.joint_5, data.joint_6]
            sp = int(data.speed)
            self.mc.send_angles(angles, sp)

        sub = rospy.Subscriber('mycobot/angles_goal', MycobotSetAngles, callback=callback)
        rospy.spin()

    def sub_set_coords(self):
        def callback(data):
            angles = [data.x, data.y, data.z, data.rx, data.ry, data.rz]
            sp = int(data.speed)
            model = int(data.model)
            self.mc.send_angles(angles, sp, model)

        sub = rospy.Subscriber('mycobot/coords_goal', MycobotSetCoords, callback=callback)
        rospy.spin()

    def sub_gripper_status(self):
        def callback(data):
            if data.Status:
                self.mc.set_gripper_state(0, 80)
            else:
                self.mc.set_gripper_state(1, 80)

        sub = rospy.Subscriber('mycobot/gripper_status', MycobotGripperStatus, callback=callback)
        rospy.spin()

    def sub_pump_status(self):
        def callback(data):
            if data.Status:
                self.mc.set_basic_output(2, 0)
                self.mc.set_basic_output(5, 0)
            else:
                self.mc.set_basic_output(2, 1)
                self.mc.set_basic_output(5, 1)

        sub = rospy.Subscriber('mycobot/pump_status', MycobotPumpStatus, callback=callback)
        rospy.spin()


if __name__ == '__main__':
    mc_topics = MycobotTopics()
    mc_topics.start()
    # while True:
    #     mc_topics.pub_real_coords()
    # mc_topics.sub_set_angles()
    pass