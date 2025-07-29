#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
 
import Tkinter
import rospy
from Tkinter import *
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot

def talker():
    rospy.init_node("display", anonymous=True)

    print("Try connect real mycobot...")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    print("port: {}, baud: {}\n".format(port, baud))
    try:
        mycobot = MyCobot(port, baud)
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
    mycobot.release_all_servos()
    time.sleep(0.1)
    print("Rlease all servos over.\n")

    pub = rospy.Publisher("joint_states", JointState, queue_size = 10)
    rate = rospy.Rate(30)

    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6",
    ]
    joint_state_send.velocity = [0] * len(joint_state_send.name)
    joint_state_send.effort = []

    print("publishing ...")
    while not rospy.is_shutdown():
        joint_state_send.header.stamp = rospy.Time.now()

        angles = mycobot.get_radians()
        if angles:
            data_list = [value for value in angles]
            print(data_list)
            joint_state_send.position = data_list
            pub.publish(joint_state_send)
        rate.sleep()


if __name__ == "__main__":
    talker()
    window = Tkinter.Tk()
    window.title("Controller GUI")
    window.geometry("230x330")
    window.resizable(False, False)

    # Label Components
    coord_label1 = Label(window, text = "joint2_to_joint1")
    coord_label1.place(x = 20, y = 20, width = 100, height = 30)
    coord_label2 = Label(window, text = "joint3_to_joint2")
    coord_label2.place(x = 20, y = 70, width = 100, height = 30)
    coord_label3 = Label(window, text = "joint4_to_joint3")
    coord_label3.place(x = 20, y = 120, width = 100, height = 30)
    coord_label4 = Label(window, text = "joint5_to_joint4")
    coord_label4.place(x = 20, y = 170, width = 100, height = 30)
    coord_label5 = Label(window, text = "joint6_to_joint5")
    coord_label5.place(x = 20, y = 220, width = 100, height = 30)
    coord_label6 = Label(window, text = "joint6output_to_joint6")
    coord_label6.place(x = 20, y = 280, width = 100, height = 30)
    # Entry Components
    coord_entry1 = Entry(window, textvariable = 0.00)
    coord_entry1.place(x = 130, y = 20, width = 80, height = 30)
    coord_entry2 = Entry(window, textvariable = 0.00)
    coord_entry2.place(x = 130, y = 70, width = 80, height = 30)
    coord_entry3 = Entry(window, textvariable = 0.00)
    coord_entry3.place(x = 130, y = 120, width = 80, height = 30)
    coord_entry4 = Entry(window, textvariable = 0.00)
    coord_entry4.place(x = 130, y = 170, width = 80, height = 30)
    coord_entry5 = Entry(window, textvariable = 0.00)
    coord_entry5.place(x = 130, y = 220, width = 80, height = 30)
    coord_entry6 = Entry(window, textvariable = 0.00)
    coord_entry6.place(x = 130, y = 270, width = 80, height = 30)

    window.mainloop()