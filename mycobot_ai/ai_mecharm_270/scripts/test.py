#!/usr/bin/env python2
# -*- coding:utf-8 -*-
from pymycobot.mycobot import MyCobot
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
import time,os

mc = MyCobot(os.popen("ls /dev/ttyUSB*").readline()[:-1], 115200)
# mc = MyCobot(os.popen("ls /dev/ttyACM*").readline()[:-1], 115200)

# mc = MyCobot("/dev/ttyAMA0", 1000000)
# mc = MyCobot(PI_PORT,PI_BAUD)


# mc.send_angles([0,0,0,0,90,0],20) # coords:[95.6, 0.5, 166.4, 179.12, -0.18, 179.46]
# time.sleep(4)

mc.send_angle(3,-15,30)

# mc.send_angles([-3.25, 17.22, -32.51, 2.37, 92.54, -36.21],30)

# mc.send_coords([92.3, -104.9, 211.4, -179.6, 28.91, 131.29], 30, 0) # above the red bucket
# time.sleep(4)

# mc.send_coords([165.0, -93.6, 201.4, -173.43, 46.23, 160.65],30,0) # above the green bucket
# time.sleep(4)

# mc.send_coords([88.1, 126.3, 193.4, 162.15, 2.23, 156.02],30,0) # above the blue bucket
# time.sleep(4)

# mc.send_coords([-5.4, 120.6, 204.6, 162.66, -6.96, 159.93],30,0) # above the gray bucket
# time.sleep(3)

# mc.send_coords([80, 0, 92, 179.12, -0.18, 179.46], 30, 0)

# mc.send_angle(3,0,25)
# print(mc.get_angles())
# print(mc.get_coords())

# mc.release_all_servos()
# while True:
#   print("angles:%s"% mc.get_angles())
#   print("coords:%s"% mc.get_coords())
#   print("\n")

# mc.release_all_servos()
# mc.release_servo(3)
# mc.set_servo_calibration(1)
# mc.set_servo_calibration(2)
# mc.set_servo_calibration(3)
# mc.set_servo_calibration(4) 
# mc.set_servo_calibration(5) 
# mc.set_servo_calibration(6) 


# mc.set_basic_output(2, 0)
# mc.set_basic_output(5, 0)
# time.sleep(2)
# mc.set_basic_output(2, 1)
# mc.set_basic_output(5, 1)

# print(mc.get_basic_input(2))
# print(mc.get_basic_input(5))

# import RPi.GPIO as GPIO
# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(20, GPIO.OUT)
# GPIO.setup(21, GPIO.OUT)

# GPIO.output(20, 0)
# GPIO.output(21, 0)
# time.sleep(3)

# GPIO.output(20, 1)
# GPIO.output(21, 1)