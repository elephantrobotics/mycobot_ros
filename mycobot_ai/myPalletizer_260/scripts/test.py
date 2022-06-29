# -*- coding: utf-8 -*-

# from fileinput import filename
# from genericpath import isfile
# import os
# from sys import path
# import cv2
# from PIL import Image


# # #count=0
# for file in dirs:
#    pic_dir=os.path.join(path,file)  # res中子文件夹的路径
#    print(pic_dir)
#    for i in os.listdir(pic_dir):
#       imgdir=os.path.join(pic_dir,i)
#       print(imgdir)
#    for i in os.listdir(pic_dir):
#       image_dir=os.path.join(pic_dir,i)  #res中每个子文件夹中图片的路径
#       img1 = cv2.imread(image_dir)  # 读取res中每个子文件夹中的图片
#       #count+=1
#       print(image_dir)#输出图片的路径
#print(img1)#输出图片
#print(count)#图片个数
# dir_path = os.path.dirname(__file__)
# print(dir_path)
# # path1 = os.path.split(os.path.realpath(__file__))[0]
# # print(path1)
# path3=os.path.join(os.path.split(dir_path)[0]+'/res/')

# print(path3)
# #for file in os.listdir(path3):
# pic_dir=os.path.join(path3+'red')
# # print(pic_dir)
# for i in os.listdir(pic_dir):
#    img=os.path.join(pic_dir,i)
#    print(img)
# print(os.getcwd()+'/mycobot_ai/res/red')

# list1 = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]]
# print(list1[0][0])
# for i, v in enumerate(list1):
#     print(i)

# res = []
# for i in list1:
#     res.append(i)
# print(res)

# # for i in list1:
# #    print(i)

#    color=i
#    print(color)
# for color in range(0,4):
#    print(color)

from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
import time

# mc = MyCobot("/dev/ttyACM0", 115200)

mc = MyPalletizer("/dev/ttyAMA0", 1000000)
# mc.send_angles([0,0,0,0,0,0], 20)

# time.sleep(2)
# move_coords = [
#         [120.1, -141.6, 240.9, -173.34, -8.15, -110.11],  # above the red bucket
#         # above the green bucket
#         #[208.2, -127.8, 260.9, -157.51, -17.5, -71.18],
#         [205.6, -130.5, 263.0, -150.99, -0.07, -107.35],
#         [209.7, -18.6, 230.4, -168.48, -9.86, -39.38],
#         [196.9, -64.7, 232.6, -166.66, -9.44, -52.47],
#         [126.6, -118.1, 305.0, -157.57, -13.72, -75.3],
#     ]

# mc.send_coords([126.6, -118.1, 305.0, -157.57, -13.72, -75.3],20,1)
# time.sleep(2)
# mc.send_coords([104.9, 176.7, 242.6, -166.66, -9.44, -52.47],20,1) # above the blue bucket
# time.sleep(2)
# mc.send_coords([-20.0, 176.7, 242.6, -166.66, -9.44, -52.47],20,1)  # abobe the gray bucket
# time.sleep(2)
# mc.send_coords([120.1,151.6,250.0,-173.34,-8.15,-110.11],20,1)
# time.sleep(2)
# mc.send_coords([104.9, 176.7, 242.6, -166.66, -9.44, -52.47],20,1)

# mc.send_angles([-26.11, -6.94, -55.01, -24.16, 0, 15],20)
# mc.release_all_servos()
# time.sleep(3)
# print(mc.get_angles())
# mc.send_angles([-1.14, 3.63, -87.8, 9.05, -3.07, 15],20)
# time.sleep(2)
# print(mc.get_angles())

# mc.send_angles([-29.0, 5.88, -4.92, -76.28],25) # init the point coords:[155.3, -86.1, 218.4, -47.28]
# time.sleep(1.5)

# mc.send_angles([-47.1, 10.19, -10.1, -76.37],25) # above the red bucket; coords:[127.3, -137.1, 219.2, -29.26]
# time.sleep(1.5)

mc.send_angles([0,0,0,0],25)
time.sleep(2)

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