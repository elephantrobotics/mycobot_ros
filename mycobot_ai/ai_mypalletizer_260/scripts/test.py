# -*- coding: utf-8 -*-
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
import time,os

# mc = MyPalletizer(os.popen("ls /dev/ttyUSB*").readline()[:-1], 115200)

# mc = MyPalletizer("/dev/ttyAMA0", 1000000)
# mc.send_angles([-29.0, 5.88, -4.92, -76.28],25) # init the point coords:[155.3, -86.1, 218.4, -47.28]
# time.sleep(1.5)

# mc.send_angles([-47.1, 10.19, -10.1, -76.37],25) # above the red bucket; coords:[127.3, -137.1, 219.2, -29.26]
# time.sleep(1.5)

# mc.send_angles([0,0,-15,0],25)
# time.sleep(2)

# mc.send_coords([141.2, -142.0, 206.2, -26.8],25,1) # above the red bucket
# time.sleep(2)
# mc.send_coords([234.3, -120, 210, -48.77],25,1) # above the green bucket
# time.sleep(2)
# mc.send_coords([100.9, 159.3, 248.6, -124.27],20,1) # above the blue bucket
# time.sleep(3)
# mc.send_coords([-17.6, 161.6, 238.4, -152.31],20,1) # above the gray bucket
# time.sleep(3)

# mc.send_angle(3,0,25)
# print(mc.get_angles())
# print(mc.get_coords())

# while True:
#     print("angles:%s"%mc.get_angles())
#     print("coords:%s"%mc.get_coords())
#     print("\n")

# mc.release_all_servos()
# mc.set_servo_calibration(1)
# mc.set_servo_calibration(2)
# mc.set_servo_calibration(3)
# mc.set_servo_calibration(4) 

import sys
import numpy as np
import cv2

print("Please enter blue:")
blue = input()
print("Please enter green:")
green = input()
print("Please enter red:")
red = input()

color = np.uint8([[[blue, green, red]]])
hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

hue = hsv_color[0][0][0]

print("Lower bound is :")
print("[" + str(hue - 10) + ", 100, 100]\n")

print("Upper bound is :"),
print("[" + str(hue + 10) + ", 255, 255]")

