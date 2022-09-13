#!/usr/bin/env python2
# -*- coding:utf-8 -*-
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import tf


def talker():
    rospy.init_node("following_marker", anonymous=True)

    pub_marker = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(20)

    listener = tf.TransformListener()

    marker_ = Marker()
    marker_.header.frame_id = "/joint1"
    marker_.ns = "basic_cube"

    print("publishing ...")
    while not rospy.is_shutdown():
        now = rospy.Time.now() - rospy.Duration(0.1)

        try:
            trans, rot = listener.lookupTransform("joint1", "basic_shapes", now)
        except Exception as e:
            print(e)
            continue

        print(type(trans), trans)
        print(type(rot), rot)

        # marker 标记
        marker_.header.stamp = now
        marker_.type = marker_.CUBE
        marker_.action = marker_.ADD
        marker_.scale.x = 0.04
        marker_.scale.y = 0.04
        marker_.scale.z = 0.04

        # marker position initial. 标记初始位置
        marker_.pose.position.x = trans[0]
        marker_.pose.position.y = trans[1]
        marker_.pose.position.z = trans[2]
        marker_.pose.orientation.x = rot[0]
        marker_.pose.orientation.y = rot[1]
        marker_.pose.orientation.z = rot[2]
        marker_.pose.orientation.w = rot[3]

        marker_.color.a = 1.0
        marker_.color.g = 1.0
        pub_marker.publish(marker_)

        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
