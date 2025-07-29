#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot.mycobot import MyCobot
import threading
import Tkinter
from Tkinter import *
from pymycobot.common import ProtocolCode
import math

joint_names = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

change_list = []

def talker():
    rospy.init_node("display", anonymous=True)

    global mycobot

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

    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    # pub_marker = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    pub_command = rospy.Publisher("/mycobot_position_controller/command", JointTrajectory, queue_size=10)
    rate = rospy.Rate(30)  # 30hz

    # pub joint state
    joint_state_send = JointState()
    joint_state_send.header = Header()
    joint_state_send.name = joint_names
    joint_state_send.velocity = [0] * len(joint_state_send.name)
    joint_state_send.effort = []

    # marker_ = Marker()
    # marker_.header.frame_id = "/joint1"
    # marker_.ns = "my_namespace"

    print("publishing ...")

    global coord_entry1, coord_entry2, coord_entry3, coord_entry4, coord_entry5, coord_entry6, prev_angles
    global coord_scale1, coord_scale2, coord_scale3, coord_scale4, coord_scale5, coord_scale6
    prev_angles = []
    while not rospy.is_shutdown():
        joint_state_send.header.stamp = rospy.Time.now()

        # angles = mycobot.get_radians()
        get_angles = mycobot._mesg(ProtocolCode.GET_ANGLES, has_reply=True)
        if get_angles is None:    continue
        angles = [round(angle * (math.pi / 180), 3) for angle in get_angles]
        if angles:
            data_list = [value for value in angles]

            joint_state_send.position = data_list
            pub.publish(joint_state_send)

            # print(data_list[0])
            # print(data_list[1])
            if len(prev_angles) == 0:
                prev_angles = data_list
                coord_entry1.insert(Tkinter.END, round(data_list[0] * 180 / math.pi, 2))
                coord_entry2.insert(Tkinter.END, round(data_list[1] * 180 / math.pi, 2))
                coord_entry3.insert(Tkinter.END, round(data_list[2] * 180 / math.pi, 2))
                coord_entry4.insert(Tkinter.END, round(data_list[3] * 180 / math.pi, 2))
                coord_entry5.insert(Tkinter.END, round(data_list[4] * 180 / math.pi, 2))
                coord_entry6.insert(Tkinter.END, round(data_list[5] * 180 / math.pi, 2))
                coord_scale1.set(int(data_list[0] * 180 / math.pi))
                coord_scale2.set(int(data_list[1] * 180 / math.pi))
                coord_scale3.set(int(data_list[2] * 180 / math.pi))
                coord_scale4.set(int(data_list[3] * 180 / math.pi))
                coord_scale5.set(int(data_list[4] * 180 / math.pi))
                coord_scale6.set(int(data_list[5] * 180 / math.pi))
                continue
            if not prev_angles[0] == data_list[0]:
                coord_entry1.delete(0, Tkinter.END)
                coord_entry1.insert(Tkinter.END, round(data_list[0] * 180 / math.pi, 2))
                # coord_scale1.set(int(data_list[0] * 180 / math.pi))
            if not prev_angles[1] == data_list[1]:
                coord_entry2.delete(0, Tkinter.END)
                coord_entry2.insert(Tkinter.END, round(data_list[1] * 180 / math.pi, 2))
                # coord_scale2.set(int(data_list[1] * 180 / math.pi))
            if not prev_angles[2] == data_list[2]:
                coord_entry3.delete(0, Tkinter.END)
                coord_entry3.insert(Tkinter.END, round(data_list[2] * 180 / math.pi, 2))
                # coord_scale3.set(int(data_list[2] * 180 / math.pi))
            if not prev_angles[3] == data_list[3]:
                coord_entry4.delete(0, Tkinter.END)
                coord_entry4.insert(Tkinter.END, round(data_list[3] * 180 / math.pi, 2))
                # coord_scale4.set(int(data_list[3] * 180 / math.pi))
            if not prev_angles[4] == data_list[4]:
                coord_entry5.delete(0, Tkinter.END)
                coord_entry5.insert(Tkinter.END, round(data_list[4] * 180 / math.pi, 2))
                # coord_scale5.set(int(data_list[4] * 180 / math.pi))
            if not prev_angles[5] == data_list[5]:
                coord_entry6.delete(0, Tkinter.END)
                coord_entry6.insert(Tkinter.END, round(data_list[5] * 180 / math.pi, 2))
                # coord_scale6.set(int(data_list[5] * 180 / math.pi))
            prev_angles = data_list

            global change_list
            if len(change_list) != 0:
                # print(change_list)
                mycobot.send_angles(change_list, 25)
                change_list = []

            # Create JointTrajectory message
            traj_msg = JointTrajectory()
            traj_msg.header.stamp = rospy.Time.now()
            traj_msg.joint_names = joint_names

            point = JointTrajectoryPoint()
            point.positions = data_list
            point.velocities = [0.0] * len(joint_names)
            point.time_from_start = rospy.Duration(0.1)  # Set a small duration to continuously update

            traj_msg.points.append(point)
            pub_command.publish(traj_msg)

            # coords = mycobot.get_coords()

            # # Marker
            # marker_.header.stamp = rospy.Time.now()
            # marker_.type = marker_.SPHERE
            # marker_.action = marker_.ADD
            # marker_.scale.x = 0.04
            # marker_.scale.y = 0.04
            # marker_.scale.z = 0.04

            # if not coords:
            #     coords = [0, 0, 0, 0, 0, 0]
            #     rospy.loginfo("error [101]: cannot get coord values")

            # marker_.pose.position.x = coords[1] / 1000 * -1
            # marker_.pose.position.y = coords[0] / 1000
            # marker_.pose.position.z = coords[2] / 1000

            # marker_.color.a = 1.0
            # marker_.color.g = 1.0
            # pub_marker.publish(marker_)

        rate.sleep()

def move_arm():
    global prev_angles, change_list
    global coord_scale1, coord_scale2, coord_scale3, coord_scale4, coord_scale5, coord_scale6
    new_data_list = [float(coord_entry1.get()),float(coord_entry2.get()),float(coord_entry3.get()),float(coord_entry4.get()),float(coord_entry5.get()),float(coord_entry6.get())]
    new_data_list = [round(num, 2) for num in new_data_list]
    # print("new_data_list %s" % new_data_list)
    prev_angles = new_data_list
    change_list = new_data_list
    coord_scale1.set(int(new_data_list[0]))
    coord_scale2.set(int(new_data_list[1]))
    coord_scale3.set(int(new_data_list[2]))
    coord_scale4.set(int(new_data_list[3]))
    coord_scale5.set(int(new_data_list[4]))
    coord_scale6.set(int(new_data_list[5]))
    # print("mycobot_angles %s" % mycobot.angles)

def controller_GUI():
    window = Tkinter.Tk()
    window.title("Controller GUI")
    window.geometry("480x440")
    window.resizable(False, False)

    def apply_cur_scale(value):
        global change_list, prev_angles
        new_data_list = [float(coord_scale1.get()),float(coord_scale2.get()),float(coord_scale3.get()),float(coord_scale4.get()),float(coord_scale5.get()),float(coord_scale6.get())]
        new_data_list = [round(num, 2) for num in new_data_list]
        prev_angles = new_data_list
        change_list = new_data_list

    # Label Components
    coord_label1 = Label(window, text = "joint2_to_joint1")
    coord_label1.place(x = 20, y = 20, width = 150, height = 50)
    coord_label2 = Label(window, text = "joint3_to_joint2")
    coord_label2.place(x = 20, y = 80, width = 150, height = 50)
    coord_label3 = Label(window, text = "joint4_to_joint3")
    coord_label3.place(x = 20, y = 140, width = 150, height = 50)
    coord_label4 = Label(window, text = "joint5_to_joint4")
    coord_label4.place(x = 20, y = 200, width = 150, height = 50)
    coord_label5 = Label(window, text = "joint6_to_joint5")
    coord_label5.place(x = 20, y = 260, width = 150, height = 50)
    coord_label6 = Label(window, text = "joint6output_to_joint6")
    coord_label6.place(x = 20, y = 320, width = 150, height = 50)
    # Entry Components
    global coord_entry1, coord_entry2, coord_entry3, coord_entry4, coord_entry5, coord_entry6
    coord_entry1 = Entry(window)
    coord_entry1.place(x = 180, y = 30, width = 80, height = 30)
    coord_entry2 = Entry(window)
    coord_entry2.place(x = 180, y = 90, width = 80, height = 30)
    coord_entry3 = Entry(window)
    coord_entry3.place(x = 180, y = 150, width = 80, height = 30)
    coord_entry4 = Entry(window)
    coord_entry4.place(x = 180, y = 210, width = 80, height = 30)
    coord_entry5 = Entry(window)
    coord_entry5.place(x = 180, y = 270, width = 80, height = 30)
    coord_entry6 = Entry(window)
    coord_entry6.place(x = 180, y = 330, width = 80, height = 30)
    # Scale Components
    global coord_scale1, coord_scale2, coord_scale3, coord_scale4, coord_scale5, coord_scale6
    coord_scale1 = Scale(window, orient=HORIZONTAL, from_=-168, to=168, command=apply_cur_scale)
    coord_scale1.place(x = 280, y = 20, width = 180, height = 50)
    coord_scale2 = Scale(window, orient=HORIZONTAL, from_=-135, to=135, command=apply_cur_scale)
    coord_scale2.place(x = 280, y = 80, width = 180, height = 50)
    coord_scale3 = Scale(window, orient=HORIZONTAL, from_=-150, to=150, command=apply_cur_scale)
    coord_scale3.place(x = 280, y = 140, width = 180, height = 50)
    coord_scale4 = Scale(window, orient=HORIZONTAL, from_=-145, to=145, command=apply_cur_scale)
    coord_scale4.place(x = 280, y = 200, width = 180, height = 50)
    coord_scale5 = Scale(window, orient=HORIZONTAL, from_=-165, to=165, command=apply_cur_scale)
    coord_scale5.place(x = 280, y = 260, width = 180, height = 50)
    coord_scale6 = Scale(window, orient=HORIZONTAL, from_=-180, to=180, command=apply_cur_scale)
    coord_scale6.place(x = 280, y = 320, width = 180, height = 50)

    # Button
    change_button = Button(window, command = move_arm, text = "Set Angles")
    change_button.place(x = 140, y = 380, width = 200, height = 40)

    window.mainloop()

if __name__ == "__main__":
    controller_thread = threading.Thread(target=controller_GUI)
    controller_thread.start()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    controller_thread.join()