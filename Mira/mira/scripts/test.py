# encoding=utf-8
import time
from pymycobot.mira import Mira
from pymycobot.mycobot import MyCobot

# ma = Mira('/dev/ttyUSB0')
# ma.go_zero()

# ma.set_angles([0, 0, 0], 50)
# time.sleep(3)
# print(ma.get_angles_info())

pose_angles = [-88.85, 0.52, -85.69, 0.0, 89.82, 0.08]
pose_coords = [-83.6, -220.2, 261.7, -175.16, -0.16, -178.95]

# 初始化位置
radians_list = [1.57, 0.0, -1.41, 0, 1.47, 0.0]
angles_list = [89.64, 0.26, -80.68, 0.0, 84.28, 0.08]
coords_list = [95.6, 221.6, 280.4, -170.36, -5.61, -1.39]
# 目标位置
radians_end = [-1.57, 0.0, -1.41, 0, 1.47, 0.0]
angles_end = [-89.56, 0.26, -80.68, 0.0, 84.28, 0.08]
coords_end = [-92.5, -222.9, 280.4, -170.36, -5.61, 179.39]
mc = MyCobot('/dev/ttyUSB0', 115200)
# mc.set_tool_reference(-50,0,0,0,0,0)
# mc.set_end_type(1)

# mc.send_angles([0, 30, -50, -40, 0, 0, 50], 30)
# mc.send_angles([0, 30, -50, -40, 90, 0], 30)
# for _ in range(10):
#     mc.send_angles(angles_list, 30)
    # time.sleep(5)
    # mc.send_angles(angles_end, 30)
    # time.sleep(5)
    # print(mc.get_angles())
i = 1
# while i< 10:
#     # mc.send_radians(radians_list, 20)
#     # time.sleep(10)
#     print(mc.get_coords())
    # print(mc.get_angles())
    # time.sleep(2)
    # mc.send_radians(radians_end, 20)
    # time.sleep(10)
    # print(mc.get_coords())
    # print(mc.get_angles())
    
mc.send_radians(radians_list, 20)
    # time.sleep(10)
# print(mc.get_coords())
# print(mc.get_angles())
time.sleep(5)
mc.send_radians(radians_end, 20)

while i< 10:
    # mc.send_radians(radians_list, 20)
    # time.sleep(10)
    print(mc.get_coords())