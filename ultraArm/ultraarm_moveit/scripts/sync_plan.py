#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import JointState
import pymycobot
from packaging import version
# min low version require
MAX_REQUIRE_VERSION = '3.9.1'
current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    from pymycobot.ultraArmP340 import ultraArmP340
    class_name = 'new'
else:
    from pymycobot.ultraArm import ultraArm
    class_name = 'old'
    print("Note: This class is no longer maintained since v3.6.0, please refer to the project documentation: https://github.com/elephantrobotics/pymycobot/blob/main/README.md")



ua = None


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    # print(data.position)
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        data_list.append(radians_to_angles)
        
    rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
    ua.set_angles(data_list, 25)


def listener():
    global ua
    rospy.init_node("control_slider", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyUSB0") # Select connected device. 选择连接设备
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    if class_name == 'old':
        ua = ultraArm(port, baud)
    else:
        ua = ultraArmP340(port, baud)
    ua.power_on()
    ua.go_zero()
    
    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
