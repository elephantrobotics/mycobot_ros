from time import time
from pymycobot.mybuddy import MyBuddy
import rospy
import time
import os

from pymycobot.mybuddysocket import MyBuddySocket
import time

mb = MyBuddySocket('192.168.123.219',9000)
mb.connect('/dev/ttyACM0','115200')
# port = rospy.get_param("~port", os.popen("ls /dev/ttyACM*").readline()[:-1])
# baud = rospy.get_param("~baud", 115200)
# print(port, baud)
# mb = MyBuddy(port, baud)

# mb.set_free_mode(2)
# mb.power_on()
# mb.release_all_servos()
# mb.set_servo_calibration(3,1)

# i = 1
# while i < 7:
#     mb.release_servo(2,i)
#     i += 1
#     time.sleep(1)

'''
------------get info--------------
'''
print(mb.get_radians())
# print(mb.get_encoders(3))
# print(mb.get_encoder(1,1))
# print(mb.get_angles(1))
# print(mb.get_coords(1))
# print(mb.get_coords(2))
# print("\n")

'''
------------send info--------------
'''
# mb.send_radians(2,[-0.136, -0.462, -0.62, 0.813, 0.218, -0.071],30)
# mb.send_angles(2,[6.32, -94.39, 13.62, -15.38, -94.21, 62.84],30,1)
# time.sleep(2)
print(mb.get_angles(1))
# mb.send_angles(1,[0,0,0,0,0,0],30,0)
# mb.send_angle(1,3,30,30)
# mb.set_encoder(3,1,2048,1)
time.sleep(1)
print(mb.get_angles(2))
time.sleep(1)
print(mb.get_angles(3))