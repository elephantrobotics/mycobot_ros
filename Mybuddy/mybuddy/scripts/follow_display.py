#!/usr/bin/env python2
import time
import os
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from pymycobot.mybuddy import MyBuddy
import math

def talker():
    rospy.init_node("display", anonymous=True)

    print("Try connect real mybuddy...")
    port = rospy.get_param("~port", os.popen("ls /dev/ttyACM*").readline()[:-1])
    baud = rospy.get_param("~baud", 115200)
    print("port: {}, baud: {}\n".format(port, baud))
    try:
        mb = MyBuddy(port, baud)
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
        # "joint2_to_joint1",
        # "joint3_to_joint2",
        # "joint4_to_joint3",
        # "joint5_to_joint4",
        # "joint6_to_joint5",
        # "joint6output_to_joint6",
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
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    marker_ = Marker()
    marker_.header.frame_id = "/base_link1"
    marker_.ns = "my_namespace"

    print("joint_state_send:%s" % joint_state_send)

    print("publishing ...")
    while not rospy.is_shutdown():
        joint_state_send.header.stamp = rospy.Time.now()

        # =======left_radians=======
        lencoder = mb.get_encoders(1)
        left_radians = []
        for index, value in enumerate(lencoder):
            value = (value-2048)*2*math.pi/4096
            left_radians.append(value)

        # left_radians = mb.get_radians(1)
        # print("left_radians: %s" % left_radians)
        # if not left_radians:
        #     left_radians = [-0.008, 0.073, -0.008, 0.162, -0.479, 0.767]
            # print("set left_radians: %s" % left_radians)
        # elif left_radians:
        #     print("left_radians: %s" % left_radians)
        
        # =======right_radians=======
        rencoder = mb.get_encoders(2)
        right_radians = []
        for index, value in enumerate(rencoder):
            value = (value-2048)*2*math.pi/4096
            right_radians.append(value)

        # right_radians = mb.get_radians(2)
        # # print("right_radians: %s" % right_radians)
        # if not right_radians:
        #     right_radians = [-0.008, -0.073, -0.008, 0.162, -0.479, 0.767]
        #     print("set right_radians: %s" % right_radians)
        # elif right_radians:
        #     print("right_radians: %s" % right_radians)

        # =======waist_radian=======

        wencoder = mb.get_encoder(3,1)
        waist_radian = []
        # for index, value in enumerate(wencoder):
        #     value = (value-2048)*2*math.pi/4096
        waist_radian.append((wencoder-2048)*2*math.pi/4096)
        # print("waist_radian: %s "%waist_radian)
        # if not waist_radian:
        #     waist_radian = [0]
        #     print("set waist_radian:%s" % waist_radian)
            
        mb.set_encoder(3,1,waist_radian[0]*4096/(2*math.pi)+2048,1)


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
        print("left_coords: %s" % left_coords)
        if not left_coords:
            # rospy.loginfo("error [101]: can not get left_coord values")
            left_coords = [-50.4, 63.4, 411.6, -91.23, -0.08, -90.08]
            # print("set lc:",left_coords)

        # =======right_coords=======
        right_coords = mb.get_coords(2)
        print("right_coords: %s " % right_coords)
        if not right_coords:
            # rospy.loginfo("error [101]: can not get right_coords values")
            right_coords = [50.4, -63.4, 411.6, -91.23, -0.08, -90.08]
        #     # print("set rc:",right_coords)

        right_coords = [50.4, -63.4, 411.6, -91.23, -0.08, -90.08]

        # coords = left_coords + right_coords
        # print("all_coords:%s" % coords) 

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
