#!/usr/bin/env python
import time
import os
import sys
import signal
import threading
import math
from itertools import izip_longest # TODO: python3 is zip_longest
zip_longest = izip_longest

import rospy
from pymycobot.mycobot import MyCobot
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import SetBool, Empty
import tf


class MycobotInterface(object):

    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 115200)
        rospy.loginfo("Connect mycobot on %s,%s" % (port, baud))
        self.mc = MyCobot(port, baud)
        self.lock = threading.Lock()

        self.joint_angle_pub = rospy.Publisher("joint_state", JointState, queue_size=5)
        self.real_angles = None
        self.joint_command_sub = rospy.Subscriber("joint_command", JointState, self.jointCommandCb)

        self.pub_end_coord = rospy.get_param("~pub_end_coord", False)
        if self.pub_end_coord:
            self.end_coord_pub = rospy.Publisher("end_coord", PoseStamped, queue_size=5)

        self.servo_srv = rospy.Service("set_servo", SetBool, self.setServoCb)
        self.open_gripper_srv = rospy.Service("open_gripper", Empty, self.openGripperCb)
        self.close_gripper_srv = rospy.Service("close_gripper", Empty, self.closeGripperCb)

    def run(self):

        r = rospy.Rate(rospy.get_param("~joint_state_rate", 20.0)) # hz

        while not rospy.is_shutdown():

            # get real joint from MyCobot
            self.real_angles = self.mc.get_angles()
            if self.real_angles:
                rospy.logdebug_throttle(1.0, "get real angles from mycobot")

                msg = JointState()
                msg.header.stamp = rospy.get_rostime()

                for i, ang in enumerate(self.real_angles):
                   msg.name.append('joint' + str(i+1))
                   msg.position.append(ang / 180.0 * math.pi)
                self.joint_angle_pub.publish(msg)

            if self.pub_end_coord:
                coords = self.mc.get_coords()
                if coords:
                    msg = PoseStamped
                    msg.header.stamp = rospy.get_rostime()
                    msg.pose.position.x = coords[0]
                    msg.pose.position.y = coords[1]
                    msg.pose.position.z = coords[2]
                    q = tf.transformations.quaternion_from_euler(coords[3], coords[4], coords[5])
                    msg.poseq.quaternion.x = q[0]
                    msg.poseq.quaternion.y = q[1]
                    msg.poseq.quaternion.z = q[2]
                    msg.poseq.quaternion.w = q[3]
                    self.end_coord_pub.publish(msg)

            r.sleep()

    def jointCommandCb(self, msg):
        angles = self.real_angles
        vel = 50 # hard-coding
        for n, p, v in zip_longest(msg.name, msg.position, msg.velocity):
            id = int(n[-1]) - 1
            if 'joint' in n and id >= 0 and id < len(angles):
                if math.fabs(p) < 190.0 / 180 * math.pi: # 190 should be  retrieved from API
                    angles[id] = p * 180 / math.pi
                else:
                    rospy.logwarn("%s exceeds the limit, %f", n, p)
            if v:
                v = v * 180 / math.pi
                if v < vel:
                    vel = v

        print(angles, vel)
        self.lock.acquire()
        self.mc.send_angles(angles, vel)
        self.lock.release()

    def setServoCb(self, req):
        if req.data:
            self.lock.acquire()
            self.mc.send_angles(self.real_angles, 0)
            self.lock.release()
            rospy.loginfo("servo on")
        else:
            self.lock.acquire()
            self.mc.release_all_servos()
            self.lock.release()
            rospy.loginfo("servo off")

        return SetBool(True, "")

    def openGripperCb(self, req):
        self.lock.acquire()
        self.mc.set_gripper_state(0, 80)
        self.lock.release()
        rospy.loginfo("open gripper")

    def closeGripperCb(self, req):
        self.lock.acquire()
        self.mc.set_gripper_state(1, 80)
        self.lock.release()
        rospy.loginfo("close gripper")



if __name__ == "__main__":
    rospy.init_node("mycobot_topics")
    mc_inteface = MycobotInterface()
    mc_inteface.run()
    pass
