from pymycobot.mira import Mira
import os
import time
port = "/dev/ttyUSB0"
mc = Mira(port)
mc.go_zero()


radians_chushi= [0.34, 0, 0]
chushi_angles = [19.48, 0.0, 0.0]
chishi_coords = [165.93, 58.69, 120.0]

radians_pre = [0.0, 0.09, 0.53]
pre_angles = [0.0, 5.16, 30.37]
pre_coords = [170.33, 0.0, 58.84]

radians_zhuaqu = [0.0, 0.33, 0.87]
zhuaqu_angles = [0.0, 18.91, 49.85]
zhuaqu_coords = [172.26, 0.0, 21.8]

radians_qilai = [0.0, 0.0, 0.0]
qilai_angles = [0.0, 0.0, 0.0]
qilai_coords = [176.0, 0.0, 120.0]


move_blue = [-1.01, 0.22, 0.0]
blue_angles = [-57.87, 12.61, 0.0]
blue_coords = [107.54, -171.23, 117.11]

move_gray = [-1.61, 0.0, 0.0]
gray_angles = [-92.25, 0.0, 0.0]
gray_coords = [-6.91, -175.86, 120.0]

move_green = [0.48, 0.61, 0]
green_angles = [27.5, 34.95, 0.0]
green_coords = [217.09, 113.01, 98.36]

move_red = [0.81, 0.0, 0.0]
red_angles = [46.41, 0.0, 0.0]
red_coords = [121.35, 127.48, 120.0]


radians_chushi= [0.34, 0, 0]
chushi_angles = [19.48, 0.0, 0.0]
chishi_coords = [165.93, 58.69, 120.0]

# mc.set_radians(radians_chushi, 50)
# time.sleep(3)
# print('chushi_angles:', mc.get_angles_info())
# print('chishi_coords:', mc.get_coords_info())

# mc.set_radians(radians_pre, 50)
# time.sleep(3)
# print('pre_angles:', mc.get_angles_info())
# print('pre_coords:', mc.get_coords_info())

# mc.set_radians(radians_zhuaqu, 50)
# time.sleep(3)
# print('zhuaqu_angles:', mc.get_angles_info())
# print('zhuaqu_coords:', mc.get_coords_info())
# # mc.set_gpio_state(0)

# mc.set_radians(radians_qilai, 50)
# time.sleep(3)
# print('qilai_angles:', mc.get_angles_info())
# print('qilai_coords:', mc.get_coords_info())

# mc.set_radians(move_blue, 50)
# time.sleep(4)
# print('blue_angles:', mc.get_angles_info())
# print('blue_coords:', mc.get_coords_info())
# # mc.set_gpio_state(1)
# time.sleep(2)

# mc.set_radians(radians_chushi, 50)
# time.sleep(5)
# print('chushi_angles:', mc.get_angles_info())
# print('chishi_coords:', mc.get_coords_info())

# mc.set_radians(move_gray, 50)
# time.sleep(6)
# print('gray_angles:', mc.get_angles_info())
# print('gray_coords:', mc.get_coords_info())

# mc.set_radians(radians_chushi, 50)
# time.sleep(2)

mc.set_radians(move_green, 50)
time.sleep(4)
print('green_angles:', mc.get_angles_info())
print('green_coords:', mc.get_coords_info())

mc.set_radians(radians_chushi, 50)
time.sleep(2)

mc.set_radians(move_red, 50)
time.sleep(4)
print('red_angles:', mc.get_angles_info())
print('red_coords:', mc.get_coords_info())

mc.set_radians(radians_chushi, 50)
