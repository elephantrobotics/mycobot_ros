#!/usr/bin/env python3
import time
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from pymycobot.mercury import Mercury


def talker():
    rospy.init_node("display", anonymous=True)

    print("Try connect real Mercury...")
    port1 = rospy.get_param("~port1", "/dev/ttyS0")
    port2 = rospy.get_param("~port2", "/dev/ttyTHS1")
    baud = rospy.get_param("~baud", 115200)
    print("port1: {}, baud: {}\n".format(port1, baud))
    print("port2: {}, baud: {}\n".format(port2, baud))
    try:
        # left arm
        cx1 = Mercury(port1, baud)
        time.sleep(0.02)
        # right arm
        cx2 = Mercury(port2, baud)
    except Exception as e:
        print(e)
        print(
            """\
            \rTry connect Mercury failed!
            \rPlease check wether connected with Mercury.
            \rPlease chckt wether the port or baud is right.
        """
        )
        exit(1)
    cx1.release_all_servos()
    time.sleep(0.02)
    cx2.release_all_servos()
    time.sleep(0.1)
    print("Rlease all servos over.\n")

    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    pub_marker = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(30)  # 30hz

    # pub joint state
    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = [
        "joint1_L",
        "joint2_L",
        "joint3_L",
        "joint4_L",
        "joint5_L",
        "joint6_L",
        "joint7_L",
        "joint1_R",
        "joint2_R",
        "joint3_R",
        "joint4_R",
        "joint5_R",
        "joint6_R",
        "joint7_R",
        "eye",
        "head",
        "body",
    ]
    
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    marker_ = Marker()
    marker_.header.frame_id = "/base"
    marker_.ns = "my_namespace"

    print("publishing ...")
    while not rospy.is_shutdown():
        joint_state_send.header.stamp = rospy.Time.now()

        left_angles = cx1.get_angles()
        right_angles = cx2.get_angles()
        eye_angle = cx2.get_angle(11)
        head_angle = cx2.get_angle(12)
        body_angle = cx2.get_angle(13)
        
        print('left:', left_angles)
        print('right:', right_angles)
        print('eye:', eye_angle)
        print('head:', head_angle)
        print('body:', body_angle)
        
        all_angles = left_angles + right_angles + eye_angle + head_angle + body_angle
        data_list = []
        for index, value in enumerate(all_angles):
            radians = math.radians(value)
            data_list.append(radians)

        # rospy.loginfo('{}'.format(data_list))
        joint_state_send.position = data_list

        pub.publish(joint_state_send)

        left_coords = cx1.get_coords()
        
        right_coords = cx2.get_coords()
        
        eye_coords = cx2.get_angle(11)
        
        head_coords = cx2.get_angle(12)
        
        body_coords = cx2.get_angle(13)
        
        
        # marker
        marker_.header.stamp = rospy.Time.now()
        marker_.type = marker_.SPHERE
        marker_.action = marker_.ADD
        marker_.scale.x = 0.04
        marker_.scale.y = 0.04
        marker_.scale.z = 0.04

        # marker position initial.标记位置初始
        # print(coords)
        # if not coords:
        #     coords = [0, 0, 0, 0, 0, 0]
        #     rospy.loginfo("error [101]: can not get coord values")

        marker_.pose.position.x = left_coords[1] / 1000 * -1
        marker_.pose.position.y = left_coords[0] / 1000
        marker_.pose.position.z = left_coords[2] / 1000
        
        time.sleep(0.02)
        
        marker_.pose.position.x = right_coords[1] / 1000 * -1
        marker_.pose.position.y = right_coords[0] / 1000
        marker_.pose.position.z = right_coords[2] / 1000
        
        time.sleep(0.02)
        
        marker_.pose.position.x = eye_coords[0] / 1000 * -1
        
        time.sleep(0.02)
        
        marker_.pose.position.x = head_coords[0] / 1000 * -1
        
        time.sleep(0.02)
        
        marker_.pose.position.x = body_coords[0] / 1000 * -1

        marker_.color.a = 1.0
        marker_.color.g = 1.0
        pub_marker.publish(marker_)

        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
