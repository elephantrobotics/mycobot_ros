#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from pymycobot import UltraArmP1
import time
import math
import rospy

ua = UltraArmP1('/dev/ttyUSB0')

ua.set_joint_disable()

# time.sleep(0.05)


while 1:
        try:
            # Update joint state header timestamp
            # joint_state_send.header.stamp = rospy.Time.now()

            # Get robot joint angles
            angles = ua.get_angles_info()
            time.sleep(0.1)
            if isinstance(angles, list) and len(angles) > 0:
                # Convert angles to radians for ROS
                data_list = [math.radians(value) for value in angles]
                # joint_state_send.position = data_list
                # pub.publish(joint_state_send)
                print('data_list:', data_list)
            else:
                rospy.logwarn("Failed to get valid angles: {}".format(angles))

            # Get robot coordinates
            coords = ua.get_coords_info()
            time.sleep(0.1)
            if not isinstance(coords, list) or len(coords) == 0 or coords == -1:
                rospy.logwarn("Failed to get valid coordinates: {}".format(coords))
                coords = [0, 0, 0, 0]  # fallback
            print('coords:', coords)
        except Exception as e:
            import traceback
            e = traceback.format_exc()
            print(e)