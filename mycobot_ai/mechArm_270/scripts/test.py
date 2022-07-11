#!/usr/bin/env python2
# -*- coding:utf-8 -*-
from pymycobot.mycobot import MyCobot
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
import time,os

# mc = MyPalletizer(os.popen("ls /dev/ttyUSB*").readline()[:-1], 115200)
mc = MyCobot("/dev/ttyAMA0", 1000000)
# mc = MyPalletizer("/dev/ttyAMA0", 1000000)


mc.send_angles([0,0,0,0,90,0],30) # coords:[95.6, 0.5, 166.4, 179.12, -0.18, 179.46]
# mc.send_angles([-3.25, 17.22, -32.51, 2.37, 92.54, -36.21],30)

time.sleep(4)

# mc.send_coords([92.3, -104.9, 211.4, -179.6, 28.91, 131.29], 30, 0) # above the red bucket
# time.sleep(4)

# mc.send_coords([165.0, -93.6, 201.4, -173.43, 46.23, 160.65],30,0) # above the green bucket
# time.sleep(4)

# mc.send_coords([88.1, 126.3, 193.4, 162.15, 2.23, 156.02],30,0) # above the blue bucket
# time.sleep(4)

# mc.send_coords([-5.4, 120.6, 204.6, 162.66, -6.96, 159.93],30,0) # above the gray bucket
# time.sleep(3)

# mc.send_coords([80, 0, 130, 0 ,0, 0], 30, 0)

# mc.send_angle(3,0,25)
# print(mc.get_angles())
# print(mc.get_coords())

# mc.release_all_servos()
# while True:
#   print("angles:%s"% mc.get_angles())
#   print("coords:%s"% mc.get_coords())
#   print("\n")

# mc.release_all_servos()
# mc.set_servo_calibration(1)
# mc.set_servo_calibration(2)
# mc.set_servo_calibration(3)
# mc.set_servo_calibration(4) 