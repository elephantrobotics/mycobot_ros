# -*- coding: utf-8 -*-
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
  # 当使用树莓派版本的mycobot时，可以引用这两个变量进行MyCobot初始化
import time

mc = MyCobot("/dev/ttyACM0", 115200)
# mc = MyCobot("/dev/ttyUSB0", 115200)
# mc = MyCobot("/dev/ttyAMA0", 1000000)

# mc.send_angles([0,0,0,0,0,0], 25)
# print(mc.get_angles())
# mc.send_angles([-7.11, -6.94, -55.01, -24.16, 0.0, -15], 30)
# time.sleep(4)
# print(mc.get_angles())

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

# mc.send_coords([145.3, -11.2, 126.6, 179.87, -3.78, -62.75], 30, 1)
# time.sleep(6)


# mc.release_all_servos()
# mc.release_servo(3)
# mc.set_servo_calibration(3)
# time.sleep(1)
# while True:
#   print("angles:%s"% mc.get_angles())
#   print("coords:%s"% mc.get_coords())
#   print("\n")
move_coords = [
            [132.2, -136.9, 200.8, -178.24, -3.72, -107.17],  # above the red bucket
            [232.5, -124.6, 212.8, -169.94, -5.88, -97.63], # green
            [115.8, 177.3, 210.6, 178.06, -0.92, -6.11], # blue
            [-6.9, 173.2, 201.5, 179.93, 0.63, 33.83], # gray
        ]

# mc.send_coords(move_coords[1],20, 1)
# mc.send_angles([-13.44, -57.3, 0.7, -22.76, -4.74, -6.76], 20)
mc.send_coords([238.8, -124.1, 204.3, -169.69, -5.52, -96.52], 20, 1)
time.sleep(3)
print(mc.get_angles())
print(mc.get_coords())