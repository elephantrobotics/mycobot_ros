#!/usr/bin/env python2
import sys
import rospy
from myCobotROS.srv import *


def test():
    rospy.wait_for_service('get_joint_angles')
    try:
        func = rospy.ServiceProxy('set_joint_coords', SetCoords)
        res = func(*[104.9000015258789, -61.79999923706055, 381.0, -
                   106.43000030517578, 2.1600000858306885, -87.80999755859375, 50, 0])
        print('res:', res)
    except:
        pass


if __name__ == '__main__':
    test()
