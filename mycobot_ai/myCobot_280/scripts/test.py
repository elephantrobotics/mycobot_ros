# -*- coding: utf-8 -*-
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
import time

# mc = MyCobot("/dev/ttyACM0", 115200)
mc = MyCobot("/dev/ttyUSB0", 115200)
# mc = MyCobot("/dev/ttyAMA0", 1000000)

# mc.send_angles([0,0,0,0,90,0], 20)
mc.send_angles([-7.11, -6.94, -55.01, -24.16, 0, -15], 30)
time.sleep(4)

# mc.send_coords([120.8, -134.4, 258.0, -172.72, -5.31, -109.09], 30, 1) # red bucket
# time.sleep(4)

# mc.send_coords([219.8, -126.4, 249.7, -158.68, -7.93, -101.6], 30, 1) # green bucket
# time.sleep(4)

# mc.send_coords([124.7, 145.3, 250.4, -173.5, -2.23, -11.7], 30, 1) # above the blue bucket
# time.sleep(4)

# mc.send_coords([14.6, 175.9, 250.4, -177.42, -0.08, 25.93], 30, 1)  # abobe the gray bucket
# time.sleep(4)

# mc.send_angles([-7.11, -6.94, -55.01, -24.16, 0, -15], 20, 0)
# mc.send_angles([1.4, 0, -53.61, -33.39, -3.51, -20.3],20)
# time.sleep(3)

# mc.send_coords([155.8, -8.4, 140, -173.3, -5.48, -57.9], 30, 1)
# time.sleep(6)

# mc.send_coords([161.5, -1.1, 115.6, -177.4, 1.09, -51.97], 30, 1)
# time.sleep(3)

# mc.release_all_servos()
# time.sleep(1)
# while True:
#   print("angles:%s"% mc.get_angles())
#   print("coords:%s"% mc.get_coords())
#   print("\n")
