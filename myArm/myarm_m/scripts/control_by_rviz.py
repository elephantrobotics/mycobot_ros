#!/usr/bin/env python3
# encoding:utf-8
# 订阅RVIZ数据，控制实际机器人运动
from pymycobot import MyArmM
from sensor_msgs.msg import JointState
import rospy
from math import pi
import time
# 1：[-174, 167]
# 2:[-92, 93]
# 3:[-91,103]
# 4:[-170, 170]
# 5:[-96,89]
# 6:[-170,170]

global mam
mam = MyArmM('/dev/ttyACM0', debug=False)
for i in range(8):
    mam.set_servo_enabled(i, 1)
    time.sleep(0.2)

def callback_fun(msg): # 回调函数
    # print(type(msg.position))
    angles = list(msg.position)
    angles.pop(7)
    gripper_angle = angles.pop(6)
    angle = [a*180/pi for a in angles]
    angle.append(gripper_angle*(-3500))
    rospy.loginfo("The Subscriber Data:%s", angle)
    set_angle(mam, angle, 10)
    # set_angle(angle, 20)
    # for i in range(6):
        # mam.set_joint_angle(i, angle[i]*180/pi, 50) # 角度直接控制
        # set_angle_by_encoder(i, angle[i]*180/pi, 50) # 电位值控制

def main():
    rospy.init_node("pub_rviz_data", anonymous=True)
    rospy.Subscriber('/joint_states', JointState, callback_fun)
    rospy.spin()

def set_angle(mam, angles, speed):
    # 弧度转角度
    mam.set_joints_angle(angles, speed)

# def set_angle_by_encoder(servo_id, angle, speed): # 输入角度 通过电位值控制机器人
#     # 1:left 2:right 3:left 4:left 5:left 6:right 7:left
#     joint_id = {0:1, 1:2, 2:4, 3:5, 4:6, 5:7}
#     # right_axis = [0, ]
#     servo_id = joint_id[servo_id]
#     if servo_id == 2 or servo_id == 6:
#         pass
#     else:
#         angle *= -1
#     encoder = int(2024 + 2024/180*angle)
#     # encoder =
#     mam.set_servo_encoder(servo_id, encoder, speed)

if __name__ == '__main__':
    main()
