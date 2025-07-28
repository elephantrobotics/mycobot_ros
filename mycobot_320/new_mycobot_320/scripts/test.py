# -*- coding: utf-8 -*-
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
import time
import pymycobot
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '3.6.0'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot320 import MyCobot320

mc = MyCobot320("/dev/ttyACM0", 115200)
# 通过传递角度参数，让机械臂每个关节移动到对应[0, 0, 0, 0, 0, 0]的位置
mc.send_angles([0, 0, 0, 0, 0, 0], 50)

# 设置等待时间，确保机械臂已经到达指定位置
# time.sleep(2.5)

# 让关节1移动到90这个位置
# mc.send_angle(Angle.J1.value, 90, 50)
# 设置等待时间，确保机械臂已经到达指定位置

# mc.send_coords([73.2, -85.9, 495.7, -94.26, -30.17, -84.42],20,0)
# time.sleep(2)

# mc.send_angles([2.19, 0.35, -3.42, -0.61, 1.23, -30.32],20)

# num=0
# while num<5:
#     print('-------------------->1')
#     print(mc.get_angles())
#     time.sleep(1)
#     print('--------------------->2')
#     print(mc.get_coords())
#     print('----------------->3')
#     time.sleep(1)
#     print(mc.get_radians())

#     num+=1

# time.sleep(2)

# # 以下代码可以让机械臂左右摇摆
# # 设置循环次数
# num=5
# while num > 0:
#     # 让关节2移动到50这个位置
#     mc.send_angle(Angle.J2.value, 50, 50)

#     # 设置等待时间，确保机械臂已经到达指定位置
#     time.sleep(1.5)

#     # 让关节2移动到-50这个位置
#     mc.send_angle(Angle.J2.value, -50, 50)

#     # 设置等待时间，确保机械臂已经到达指定位置
#     time.sleep(1.5)


#     num 
mc.set_gripper_mode(0)
# print('11')
# time.sleep(1)
# mc.set_gripper_value(50, 50)
# time.sleep(1)
# print(mc.get_angles())

mc.set_gripper_state(0,100)
time.sleep(0.05)

angles = [[-27.64, 15.2, -53.34, -52.47, 88.76, 99.05]]
coords = [189.7, -350.6, 334.0, -170.85, -0.86, -91.24]
mc.set_color(0, 255, 0)
for i in range(4):
    coords[1] += 150
    # mc.send_coords(coords, 100, 1)
    time.sleep(2)
    mc.set_gripper_state(0,100)
    time.sleep(0.05)
    coords[2] -= 155
    # mc.send_coords(coords,100,1)
    print(mc.get_angles())
    time.sleep(3)
    mc.set_gripper_state(1,100)
    time.sleep(1)
    coords[2] += 155
    # mc.send_coords(coords,100,1)
    time.sleep(2)
    coords[2] -= 155
    # mc.send_coords(coords,100,1)
    time.sleep(2)
    mc.set_gripper_state(0,100)