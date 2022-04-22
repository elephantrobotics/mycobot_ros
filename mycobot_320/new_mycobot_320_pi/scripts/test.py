# -*- coding: utf-8 -*-
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
import time

mc = MyCobot("/dev/ttyUSB0", 115200)
# 通过传递角度参数，让机械臂每个关节移动到对应[0, 0, 0, 0, 0, 0]的位置
mc.send_angles([0, 0, 0, 0, 0, 0], 50)

# 设置等待时间，确保机械臂已经到达指定位置
time.sleep(2.5)

# 让关节1移动到90这个位置
# mc.send_angle(Angle.J1.value, 90, 50)
