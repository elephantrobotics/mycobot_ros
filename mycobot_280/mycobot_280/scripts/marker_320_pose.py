#!/usr/bin/env python2
# coding:utf-8
import rospy
from visualization_msgs.msg import Marker
import time

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
        init_angles = [0, 0.52, -85.69, 0.0, 89.82, 0.08, 20]
        start_angles = [90.64, 0.52, -85.69, 0.0, 89.82, 0.08, 20]
        end_angles = [-90.56, 0.52, -85.69, 0.0, 89.82, 0.08, 20]
        try:
            self.set_angles(*init_angles)
            time.sleep(2)
            for _ in range(50):
                self.set_angles(*start_angles)
                time.sleep(9)
                self.set_angles(*end_angles)
                time.sleep(9)
        except Exception as e:
            print(e)

def grippercallback(data):
# """callback function,回调函数"""
    global mt
    #rospy.loginfo('gripper_subscriber get date :%s', data)

    start_time = time.time()
    # Parse out the coordinate value,解析出坐标值
    # pump length: 88mm
    x = float(format(data.pose.position.x, ".2f"))
    y = float(format(data.pose.position.y, ".2f"))
    z = float(format(data.pose.position.z, ".2f"))
    print(x, y, z)
    c = mt.get_coords()
    # print(type(c), c)
    #ma = MarkerMycobot()
    Pt = [c.x, c.y, c.z]
    print('mycobot:',Pt)
    Pc = [x, y, z]
        # print('camera:',Pc)
    Pm = [0, 0]
        
    offset = [-0.045, -0.2228, 0]
    imishiro = 58.43
    Pm[0] = Pt[0] + imishiro * (Pc[1] - offset[0])
    Pm[1] = Pt[1] + imishiro * (Pc[0] - offset[1])
    print('real_marker_coords:',(Pm[0], Pm[1]))
    end_time = time.time()
  
    print("loop_time:", end_time-start_time)


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
    
