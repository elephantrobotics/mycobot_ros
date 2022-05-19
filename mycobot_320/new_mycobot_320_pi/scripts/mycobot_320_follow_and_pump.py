#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker
import time
import os

# 与 mycobot 通信的消息类型
from mycobot_communication.msg import MycobotSetAngles, MycobotSetCoords, MycobotPumpStatus


rospy.init_node("gipper_subscriber", anonymous=True)

# 控制 mycobot 的 topic，依次是角度、坐标、夹爪
angle_pub = rospy.Publisher("mycobot/angles_goal",
                            MycobotSetAngles, queue_size=5)
coord_pub = rospy.Publisher("mycobot/coords_goal",
                            MycobotSetCoords, queue_size=5)
# Judging equipment: ttyUSB* is M5；ttyACM* is wio
robot = os.popen("ls /dev/ttyUSB*").readline()

if "dev" in robot:
    Pin = [2, 5]
else:
    Pin = [20, 21]

pump_pub = rospy.Publisher("mycobot/pump_status",
                           MycobotPumpStatus, queue_size=5)

# 实例化消息对象
angles = MycobotSetAngles()
coords = MycobotSetCoords()
pump = MycobotPumpStatus()

# 与 mycobot 真实位置的偏差值
x_offset = -20
y_offset = 20
z_offset = 110

# 通过该变量限制，抓取行为只做一次
flag = False

# 为了后面比较二维码是否移动
temp_x = temp_y = temp_z = 0.0

temp_time = time.time()


def pub_coords(x, y, z, rx=-150, ry=10, rz=-90, sp=70, m=2):
    """发布坐标"""
    coords.x = x
    coords.y = y
    coords.z = z
    coords.rx = rx
    coords.ry = ry
    coords.rz = rz
    coords.speed = 70
    coords.model = m
    # print(coords)
    coord_pub.publish(coords)


def pub_angles(a, b, c, d, e, f, sp):
    """发布角度"""
    angles.joint_1 = float(a)
    angles.joint_2 = float(b)
    angles.joint_3 = float(c)
    angles.joint_4 = float(d)
    angles.joint_5 = float(e)
    angles.joint_6 = float(f)
    angles.speed = sp
    angle_pub.publish(angles)


def pub_pump(flag, Pin):
    """发布夹爪状态"""
    pump.Status = flag
    pump.Pin1 = Pin[0]
    pump.Pin2 = Pin[1]
    pump_pub.publish(pump)


def target_is_moving(x, y, z):
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
    """回调函数"""
    global flag, temp_x, temp_y, temp_z
    # rospy.loginfo('gripper_subscriber get date :%s', data)
    if flag:
        return

    # 解析出坐标值
    # pump length: 88mm
    x = float(format(data.pose.position.x * 1000, ".2f"))
    y = float(format(data.pose.position.y * 1000, ".2f"))
    z = float(format(data.pose.position.z * 1000, ".2f"))

    # 当运行时间小于 30s，或目标位置还在改变时，进行追踪行为
    if (
        time.time() - temp_time < 30
        or (temp_x == temp_y == temp_z == 0.0)
        or target_is_moving(x - x_offset, y - y_offset, z)
    ):

        x -= x_offset
        y -= y_offset
        pub_coords(x - 20, y, 280)
        time.sleep(0.1)

        temp_x, temp_y, temp_z = x, y, z
        return
    else:  # 表示目标处于静止状态，可以尝试抓取

        print(x, y, z)

        # detect heigth + pump height + limit height + offset
        x += x_offset
        y += y_offset
        z = z + 88 + z_offset

        pub_coords(x, y, z)
        time.sleep(2.5)

        # down
        for i in range(1, 17):
            pub_coords(x, y, z - i * 5, rx=-160, sp=10)
            time.sleep(0.1)

        time.sleep(2)

        pub_pump(True, Pin)
        # pump on

        pub_coords(x, y, z + 20, -165)
        time.sleep(1.5)

        pub_angles(0, 30, -50, -40, 0, 0, 50)
        time.sleep(1.5)

        put_z = 140
        pub_coords(39.4, -174.7, put_z, -177.13, -4.13, -152.59, 70, 2)
        time.sleep(1.5)

        for i in range(1, 8):
            pub_coords(39.4, -174.7, put_z - i * 5, -
                       177.13, -4.13, -152.59, 15, 2)
            time.sleep(0.1)

        pub_pump(False, Pin)

        time.sleep(0.5)

        pub_angles(0, 30, -50, -40, 0, 0, 50)
        time.sleep(1.5)

        # finally
        flag = True


def main():
    for _ in range(10):
        # pub_coords(150, 20, 220, -175, 0, -90, 70, 2)
        pub_angles(0, 30, -50, -40, 0, 0, 50)
        #     pub_angles(random.randint(-30, 30), random.randint(-30, 30), random.randint(-30, 30), random.randint(-30, 30), random.randint(-30, 30), random.randint(-30, 30), 70)
        time.sleep(0.5)

    pub_pump(False, Pin)
    # time.sleep(2.5)

    # mark 信息的订阅者
    rospy.Subscriber("visualization_marker", Marker,
                     grippercallback, queue_size=1)

    print("gripper test")
    rospy.spin()


if __name__ == "__main__":
    main()
