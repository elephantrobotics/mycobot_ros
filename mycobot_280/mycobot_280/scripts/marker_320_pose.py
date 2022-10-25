#!/usr/bin/env python2
# coding:utf-8
import rospy
from visualization_msgs.msg import Marker
import time
import math
# Type of message communicated with mycobot，与 mycobot 通信的消息类型
from mycobot_communication.srv import GetCoords, SetCoords, GetAngles, SetAngles

class MarkerMycobot():
    def __init__(self):

        rospy.init_node("gipper_subscriber", anonymous=True)

        self.model = 0
        self.speed = 20
        self.record_coords = [0, 0, 0, 0, 0, 0]
        self.connect_str()
        #self.sends_angles()
        #self.get_date()

    def connect_str(self):
        rospy.wait_for_service("get_joint_angles")
        rospy.wait_for_service("set_joint_angles")
        rospy.wait_for_service("get_joint_coords")
        rospy.wait_for_service("set_joint_coords")

        try:
            self.get_coords = rospy.ServiceProxy("get_joint_coords", GetCoords)
            self.set_coords = rospy.ServiceProxy("set_joint_coords", SetCoords)
            self.get_angles = rospy.ServiceProxy("get_joint_angles", GetAngles)
            self.set_angles = rospy.ServiceProxy("set_joint_angles", SetAngles)
        except:
            print("start error ...")
            exit(1)

    def get_date(self):
        t = time.time()
        while time.time() - t <2:
            self.res_coords = self.get_coords()

        time.sleep(0.1)
        self.record_coords = [
        round(self.res_coords.x, 2),
        round(self.res_coords.y, 2),
        round(self.res_coords.z, 2),
        round(self.res_coords.rx, 2),
        round(self.res_coords.ry, 2),
        round(self.res_coords.rz, 2),
    ]
        print(self.record_coords)
        return self.record_coords

    def sends_angles(self):
        init_angles = [0, 0.52, -85.69, 0.0, 89.82, 0.08, 5]
        start_angles = [20.64, 0.52, -85.69, 0.0, 89.82, 0.08, 5]
        end_angles = [-20.56, 0.52, -85.69, 0.0, 89.82, 0.08, 5]
        try:
            self.set_angles(*init_angles)
            time.sleep(0.01)
            for _ in range(50):
                self.set_angles(*start_angles)
                time.sleep(4.5)
                self.set_angles(*end_angles)
                time.sleep(4.5)
        except Exception as e:
            print(e)

def grippercallback(data):
# """callback function,回调函数"""
    global mt
    #rospy.loginfo('gripper_subscriber get date :%s', data)

    start_time = time.time()
    print('Start........')
    # Parse out the coordinate value,解析出坐标值
    # pump length: 88mm
    x = float(format(data.pose.position.x, ".3f"))
    y = float(format(data.pose.position.y, ".3f"))
    z = float(format(data.pose.position.z, ".3f"))
    print(x, y, z)
    c = mt.get_coords()
    b = mt.get_angles()
   
    #ma = MarkerMycobot()
    angles_data = [b.joint_1, b.joint_2, b.joint_3, b.joint_4, b.joint_5, b.joint_6]
    q1 = math.radians(angles_data[0])
    Pt = [round(c.x, 3), round(c.y, 3)]
    print('mycobot:',Pt)
    Pc = [x, y, z]
   
    Pm = [0, 0]
        
    offset = [0.009, 0.018, 0.218]
    imishiro = 86.77
    px = Pc[0] - offset[0]
    py = Pc[1] - offset[1]
    c1 = math.cos(q1)
    s1 = math.sin(q1)
    x = c1*px + py*s1
    y = px*s1 - c1*py

    Pm[0] = round(Pt[0] + imishiro*x, 3)
    Pm[1] = round(Pt[1] + imishiro*y, 3)
    print('Pm_marker_Pm:',(Pm[0], Pm[1]))
    end_time = time.time()
  
    print("loop_time:", end_time-start_time)
    print('END.......')


def main():
    global mt
    print("Start")
    mt = MarkerMycobot()
    
    print("Subscribe")
    # mark 信息的订阅者,subscribers to mark information
    rospy.Subscriber("visualization_marker", Marker,
                    grippercallback, queue_size=1)

    print("Before Loop")

    mt.sends_angles()

    print("gripper test")
    rospy.spin()


if __name__ == "__main__":
    main()
    
