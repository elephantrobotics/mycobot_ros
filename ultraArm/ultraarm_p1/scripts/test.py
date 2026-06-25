#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from pymycobot import UltraArmP1
import time
import math
import rospy

ua = UltraArmP1('/dev/ttyUSB0', baudrate=1000000, debug=True)

# print(ua.download_firmware_sd('./upstm32.bin'))
# print(ua.upgrade_restart())
# ua.set_joint_enable()
# exit()
# ua.set_joint_release()
# time.sleep(2)
# ua.clear_error_status()
ua.collision_unlock()
print(ua.go_home())
# time.sleep(0.05)
exit()
print(ua.set_angles([0, 0, 90], 2000, _async=False))
while 1:
    
    print(ua.set_angles([0, 0, 90], 2000, _async=False))
# print(ua.get_angles_info())
exit()

while 1:
        try:
            # Update joint state header timestamp
            # joint_state_send.header.stamp = rospy.Time.now()

            # Get robot joint angles
            angles = ua.get_angles_info()
            time.sleep(0.05)
            if isinstance(angles, list) and len(angles) > 0:
                # Convert angles to radians for ROS
                # data_list = [math.radians(value) for value in angles]
                # joint_state_send.position = data_list
                # pub.publish(joint_state_send)
                print('data_list:', angles)
            else:
                rospy.logwarn("Failed to get valid angles: {}".format(angles))

            # Get robot coordinates
            # coords = ua.get_coords_info()
            # time.sleep(0.1)
            # if not isinstance(coords, list) or len(coords) == 0 or coords == -1:
            #     rospy.logwarn("Failed to get valid coordinates: {}".format(coords))
            #     coords = [0, 0, 0, 0]  # fallback
            # print('coords:', coords)
        except Exception as e:
            import traceback
            e = traceback.format_exc()
            print(e)