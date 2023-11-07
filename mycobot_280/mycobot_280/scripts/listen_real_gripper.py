#!/usr/bin/env python3
# encoding:utf-8
# license removed for brevity
from distutils.log import error
import time
import math
import os
import fcntl

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_communication.srv import GetAngles
from pymycobot.mycobot import MyCobot
from rospy import ServiceException

mc = None

# Avoid serial port conflicts and need to be locked
def acquire(lock_file):
    open_mode = os.O_RDWR | os.O_CREAT | os.O_TRUNC
    fd = os.open(lock_file, open_mode)

    pid = os.getpid()
    lock_file_fd = None
    
    timeout = 50.0
    start_time = current_time = time.time()
    while current_time < start_time + timeout:
        try:
            # The LOCK_EX means that only one process can hold the lock
            # The LOCK_NB means that the fcntl.flock() is not blocking
            # and we are able to implement termination of while loop,
            # when timeout is reached.
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (IOError, OSError):
            pass
        else:
            lock_file_fd = fd
            break

        # print('pid waiting for lock:%d'% pid)


        time.sleep(1.0)
        current_time = time.time()
    if lock_file_fd is None:
        os.close(fd)
    return lock_file_fd


def release(lock_file_fd):
    # Do not remove the lockfile:
    fcntl.flock(lock_file_fd, fcntl.LOCK_UN)
    os.close(lock_file_fd)
    return None

def talker():
    rospy.loginfo("start ...")
    
    rospy.init_node("real_listener_gripper", anonymous=True)
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    mc = MyCobot(port, baud)
    rate = rospy.Rate(30)  # 30hz

    # pub joint state，发布关节状态
    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6",
        "gripper_controller",
    ]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    # waiting util server `get_joint_angles` enable.等待'get_joint_angles'服务启用
    rospy.loginfo("wait service")
    rospy.wait_for_service("get_joint_angles")
    
    while True:
        try:
            func = rospy.ServiceProxy("get_joint_angles", GetAngles)
            break
        except ServiceException as e:
            # pass
            # print(f'error:{e}')
            print("--------------error",e)
    
    rospy.loginfo("start loop ...")
    while not rospy.is_shutdown():
        # get real angles from server.从服务器获得真实的角度。
        res = func()
        if res.joint_1 == res.joint_2 == res.joint_3 == 0.0:
            continue
        if mc:
            lock = acquire("/tmp/mycobot_lock")
            gripper_value = mc.get_gripper_value()
            release(lock)
        if gripper_value != -1:
            gripper_value = -0.78 + round(gripper_value / 117.0, 2)
            # print(gripper_value)            
            radians_list = [
                res.joint_1 * (math.pi / 180),
                res.joint_2 * (math.pi / 180),
                res.joint_3 * (math.pi / 180),
                res.joint_4 * (math.pi / 180),
                res.joint_5 * (math.pi / 180),
                res.joint_6 * (math.pi / 180),
            ]
            radians_list.append(gripper_value)
            rospy.loginfo("res: {}".format(radians_list))

            # publish angles.发布角度
            joint_state_send.header.stamp = rospy.Time.now()
            joint_state_send.position = radians_list
            pub.publish(joint_state_send)
            rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass