#!/usr/bin/env python2
# coding:utf-8
from cv2 import goodFeaturesToTrack
from pymycobot import MyCobot
import rospy
from visualization_msgs.msg import Marker
import time
import os

# Type of message communicated with mycobot，与 mycobot 通信的消息类型
from mycobot_communication.msg import MycobotSetAngles, MycobotSetCoords, MycobotPumpStatus


rospy.init_node("gipper_subscriber", anonymous=True)

# Control the topic of mycobot, followed by angle, coordinates, gripper
# 控制 mycobot 的 topic，依次是角度、坐标、夹爪
angle_pub = rospy.Publisher("mycobot/angles_goal",
                            MycobotSetAngles, queue_size=5)
coord_pub = rospy.Publisher("mycobot/coords_goal",
                            MycobotSetCoords, queue_size=5)
# 判断设备：ttyUSB*为M5；ttyACM*为wio，Judging equipment: ttyUSB* is M5；ttyACM* is wio
robot = os.popen("ls /dev/ttyUSB*").readline()

mc = MyCobot('/dev/ttyUSB0', 115200)
mc.set_tool_reference([-50,0,0,0,0,0])
mc.set_end_type(1)


if "dev" in robot:
    Pin = [2, 5]
else:
    Pin = [20, 21]

pump_pub = rospy.Publisher("mycobot/pump_status",
                           MycobotPumpStatus, queue_size=5)

# instantiate the message object，实例化消息对象
angles = MycobotSetAngles()
coords = MycobotSetCoords()
pump = MycobotPumpStatus()

# Deviation value from mycobot's real position,与 mycobot 真实位置的偏差值
x_offset = -20
y_offset = 20
z_offset = 110

# With this variable limit, the fetching behavior is only done once
# 通过该变量限制，抓取行为只做一次
flag = False

# In order to compare whether the QR code moves later,为了后面比较二维码是否移动
temp_x = temp_y = temp_z = 0.0

temp_time = time.time()


def pub_coords(x, y, z, rx=-170, ry=-5.6, rz=-90, sp=20, m=1):
    """Post coordinates,发布坐标"""
    coords.x = x
    coords.y = y
    coords.z = z
    coords.rx = rx
    coords.ry = ry
    coords.rz = rz
    coords.speed = 20
    coords.model = m
    # print(coords)
    coord_pub.publish(coords)


def pub_angles(a, b, c, d, e, f, sp):
    """Publishing angle,发布角度"""
    angles.joint_1 = float(a)
    angles.joint_2 = float(b)
    angles.joint_3 = float(c)
    angles.joint_4 = float(d)
    angles.joint_5 = float(e)
    angles.joint_6 = float(f)
    angles.speed = sp
    angle_pub.publish(angles)



def target_is_moving(x, y, z):
    """Determine whether the target moves"""
    """判断目标是否移动"""
    count = 0
    for o, n in zip((x, y, z), (temp_x, temp_y, temp_z)):
        print(o, n)
        if abs(o - n) < 2:
            count += 1
    print(count)
    if count == 3:
        return False
    return True


def grippercallback(data):
    """callback function,回调函数"""
    global flag, temp_x, temp_y, temp_z
    # rospy.loginfo('gripper_subscriber get date :%s', data)
    if flag:
        return
   
    # robot_coords = mc.get_coords()
   
    # pub_angles(89.64, 0.52, -85.69, 0.0, 89.82, 0.08, 20)
    # time.sleep(10)
    # pub_angles(-89.56, 0.52, -85.69, 0.0, 89.82, 0.08, 20)
    # time.sleep(10)

    # i = 1
    # while i <10:
    #     robot_coords = mc.get_coords()
    
    # Parse out the coordinate value,解析出坐标值
    # pump length: 88mm
    x = float(format(data.pose.position.x, ".2f"))
    y = float(format(data.pose.position.y, ".2f"))
    z = float(format(data.pose.position.z, ".2f"))
    print(x, y, z)
    i = 1
    while i <10:
        robot_coords = mc.get_coords()
        
    if robot_coords != None:
        print(robot_coords)
        Pt = [robot_coords[0],robot_coords[1], robot_coords[2]]
    print('mycobot:',Pt)
    Pc = [x, y, z]
    print('camera:',Pc)
    Pm = [0, 0]
    
    offset = [-0.045, -0.2228, 0]
    imishiro = 58.43
    Pm[0] = Pt[0] + imishiro * (Pc[1] - offset[0])
    Pm[1] = Pt[1] + imishiro * (Pc[0] - offset[1])
    print('real_coords:',(Pm[0], Pm[1]))

    pub_angles(89.64, 0.52, -85.69, 0.0, 89.82, 0.08, 20)
    time.sleep(10)
    pub_angles(-89.56, 0.52, -85.69, 0.0, 89.82, 0.08, 20)
    time.sleep(10)

    
 

    # return Pm



def main():
    for _ in range(10):
        pub_angles(0.64, 0.52, -85.69, 0.0, 89.82, 0.08, 20)
        time.sleep(0.5)

    # mark 信息的订阅者,subscribers to mark information
    rospy.Subscriber("visualization_marker", Marker,
                     grippercallback, queue_size=1)

    print("gripper test")
    rospy.spin()


if __name__ == "__main__":
    main()
