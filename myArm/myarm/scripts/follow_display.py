#!/usr/bin/env python2
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from pymycobot.myarm import MyArm


def talker():
    rospy.init_node("display", anonymous=True)

    print("Try connect real mycobot...")
    port = rospy.get_param("~port", "/dev/ttyAMA0")
    baud = rospy.get_param("~baud", 115200)
    print("port: {}, baud: {}\n".format(port, baud))
    try:
        mycobot = MyArm(port, baud)
    except Exception as e:
        print(e)
        print(
            """\
            \rTry connect mycobot failed!
            \rPlease check wether connected with mycobot.
            \rPlease chckt wether the port or baud is right.
        """
        )
        exit(1)
    mycobot.release_all_servos(0)
    time.sleep(0.1)
    print("Rlease all servos over.\n")

    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    pub_marker = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(30)  # 30hz

    # pub joint state
    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = [
        "joint1_to_base",
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint7_to_joint6",
    ]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    marker_ = Marker()
    marker_.header.frame_id = "/base"
    marker_.ns = "my_namespace"

    print("publishing ...")
    while not rospy.is_shutdown():
        joint_state_send.header.stamp = rospy.Time.now()

        angles = mycobot.get_radians()
        data_list = []
        for index, value in enumerate(angles):
            data_list.append(value)

        # rospy.loginfo('{}'.format(data_list))
        joint_state_send.position = data_list

        pub.publish(joint_state_send)

        coords = mycobot.get_coords()

        # marker
        marker_.header.stamp = rospy.Time.now()
        marker_.type = marker_.SPHERE
        marker_.action = marker_.ADD
        marker_.scale.x = 0.04
        marker_.scale.y = 0.04
        marker_.scale.z = 0.04

        # marker position initial.标记位置初始
        # print(coords)
        if not coords:
            coords = [0, 0, 0, 0, 0, 0]
            rospy.loginfo("error [101]: can not get coord values")

        marker_.pose.position.x = coords[1] / 1000 * -1
        marker_.pose.position.y = coords[0] / 1000
        marker_.pose.position.z = coords[2] / 1000

        marker_.color.a = 1.0
        marker_.color.g = 1.0
        pub_marker.publish(marker_)

        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
