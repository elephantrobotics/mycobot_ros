#!/usr/bin/env python3
import time
import os
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from pymycobot.mybuddysocket import MyBuddySocket
import math

def talker():
    rospy.init_node("display", anonymous=True)

    print("Try connect real mybuddy...")
    ip = rospy.get_param("~ip", "192.168.123.219")
    port = rospy.get_param("~port", 9000)
    print("ip: {}, port: {}\n".format(ip, port))
    try:
        mb = MyBuddySocket(ip, port)
        mb.connect(serialport="/dev/ttyACM0", baudrate="115200")
    except Exception as e:
        print(e)
        print(
            """\
            \rTry connect mybuddy failed!
            \rPlease check wether connected with mybuddy.
            \rPlease chckt wether the port or baud is right.
        """
        )
        exit(1)

    mb.release_all_servos()
    time.sleep(0.5)
    print("Rlease all servos over.\n")

    pub = rospy.Publisher("joint_states", JointState, queue_size=20)
    pub_marker = rospy.Publisher("visualization_marker", Marker, queue_size=20)
    rate = rospy.Rate(30)  # hz

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
        "joint1_R",
        "joint2_R",
        "joint3_R",
        "joint4_R",
        "joint5_R",
        "joint6_R",
        "base_link1",

    ]
    joint_state_send.velocity = [0.0]
    joint_state_send.effort = []

    marker_ = Marker()
    marker_.header.frame_id = "/base_link1"
    marker_.ns = "my_namespace"


    print("publishing ...")
    while not rospy.is_shutdown():
        joint_state_send.header.stamp = rospy.Time.now()

        left_radians = mb.get_radians(1)

        right_radians = mb.get_radians(2)
       
        wangles = mb.get_angles(3)[0]*(math.pi/180)
        waist_radian =[]
        waist_radian.append(wangles)

        print('left:',left_radians,'right:',right_radians,'w:',waist_radian)
        
        # =======all_radians=======
        all_radians = left_radians + right_radians + waist_radian
        data_list = []
        for index, value in enumerate(all_radians):
            data_list.append(value)
            
        # rospy.loginfo('{}'.format(data_list))
        joint_state_send.position = data_list
        
        print("all_radians: %s" % data_list)
        
        pub.publish(joint_state_send)

        # =======left_coords=======
        left_coords = mb.get_coords(1)

        # =======right_coords=======
        right_coords = mb.get_coords(2)

        waist_coords = mb.get_angles(3)
       
        # marker
        marker_.header.stamp = rospy.Time.now()
        marker_.type = marker_.SPHERE
        marker_.action = marker_.ADD
        marker_.scale.x = 0.04
        marker_.scale.y = 0.04
        marker_.scale.z = 0.04

        # marker position initial
        marker_.pose.position.x = left_coords[1] / 1000 * -1
        marker_.pose.position.y = left_coords[0] / 1000
        marker_.pose.position.z = left_coords[2] / 1000

        time.sleep(0.02)

        marker_.pose.position.x = right_coords[1] / 1000 * -1
        marker_.pose.position.y = right_coords[0] / 1000
        marker_.pose.position.z = right_coords[2] / 1000

        time.sleep(0.02)

        marker_.pose.position.x = waist_coords[0] / 1000 * -1
        
        marker_.color.a = 1.0
        marker_.color.r = 0.0
        marker_.color.g = 1.0
        marker_.color.b = 0.0
        pub_marker.publish(marker_)
        rate.sleep()

        print("\n")
        

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
