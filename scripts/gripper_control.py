#!/usr/bin/env python2
# license removed for brevity


import rospy
from visualization_msgs import msg
from visualization_msgs.msg import Marker
# from pymycobot.mycobot import MyCobot
# from pymycobot.genre import Coord
# from pymycobot import PI_PORT, PI_BAUD # For raspberry pi version of mycobot.
import time
from mycobot_ros.srv import (
    GetCoords, SetCoords, GetAngles, SetAngles, GripperStatus)


set_coords = None
set_angles = None

x_offset = 0
y_offset = 30
z_offset = 60

flag = False


def grippercallback(data):
    global flag
    print(type(data))
    # rospy.loginfo('gripper_subscriber get date :%s', data)
    # print(Marker.)
    # coord_datas = mc.get_coords()
    # print(coord_datas)
    if flag:
        return


    # pump lenght: 88mm
    x = float(format(data.pose.position.x*1000, '.2f'))
    y = float(format(data.pose.position.y*1000, '.2f'))
    z = float(format(data.pose.position.z*1000, '.2f'))


    print(x, y, z)

    # detect heigth + pump height + limit height + offset
    x += x_offset
    y += y_offset
    z = z  + 88 + 25 + z_offset

    try:
        set_coords(x, y, z, -175, 0, -90, 70, 2)
        time.sleep(2.5)
    except Exception:
        pass


    for i in range(1,4):
        try:
            set_coords(x, y, z - i * 10, -175, 0, -90, 70, 2)
            time.sleep(.2)
        except Exception:
            pass
    
    # pump on
    try:
        set_coords(x, y, z + 20, -175, 0, -90, 70, 2)
        time.sleep(2.5)
    except Exception:
        pass

    # ...

    # finally
    flag = True

    




def gipper_subscriber():
    global set_coords, set_angles
    # rospy.wait_for_service('get_joint_angles')
    rospy.wait_for_service('set_joint_angles')
    # rospy.wait_for_service('get_joint_coords')
    rospy.wait_for_service('set_joint_coords')
    try:
        # get_coords = rospy.ServiceProxy('get_joint_coords', GetCoords)
        set_coords = rospy.ServiceProxy('set_joint_coords', SetCoords)
        # get_angles = rospy.ServiceProxy('get_joint_angles', GetAngles)
        set_angles = rospy.ServiceProxy('set_joint_angles', SetAngles)
    except:
        print('start error ...')
        exit(1)

    try:
        set_angles(0, 30, -50, -40, 0, 0, 50)
    except Exception:
        pass
    time.sleep(2.5)
    rospy.init_node('gipper_subscriber',anonymous=True)
    rospy.Subscriber('visualization_marker',Marker,grippercallback, queue_size=1)
    print 'gripper test'
    rospy.spin()

    
    

if __name__ == '__main__':
    gipper_subscriber()
    