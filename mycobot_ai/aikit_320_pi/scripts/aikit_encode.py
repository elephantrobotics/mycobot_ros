#encoding: UTF-8
import cv2
import numpy as np
from pymycobot.mycobot import MyCobot
import time
import os
import rospy
from visualization_msgs.msg import Marker
from moving_utils import Movement

# y轴偏移量
pump_y = -55
# x轴偏移量
pump_x = 15

class Detect_marker(Movement):
    def __init__(self):
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        
        # which robot: USB* is m5; ACM* is wio; AMA* is raspi
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        self.robot_raspi = os.popen("ls /dev/ttyAMA*").readline()[:-1]
        self.robot_jes = os.popen("ls /dev/ttyTHS1").readline()[:-1]
        
        # Creating a Camera Object
        cap_num = 0
        self.cap = cv2.VideoCapture(cap_num)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        
        # Determine the placement point of the QR code
        self.color = 0
        
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        # 摄像头的内参矩阵
        self.camera_matrix = np.array([
            [781.33379113, 0., 347.53500524],
            [0., 783.79074192, 246.67627253],
            [0., 0., 1.]])

        # 摄像头的畸变系数
        self.dist_coeffs = np.array(([[3.41360787e-01, -2.52114260e+00, -1.28012469e-03,  6.70503562e-03,
             2.57018000e+00]]))
    
        # init a node and a publisher
        rospy.init_node("encode_marker", anonymous=True)
        self.pub = rospy.Publisher('/cube', Marker, queue_size=1)

        self.marker = Marker()
        self.marker.header.frame_id = "base"
        self.marker.ns = "cube"
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.04
        self.marker.scale.y = 0.04
        self.marker.scale.z = 0.04
        self.marker.color.a = 1
        self.marker.color.r = 0.3
        self.marker.color.g = 0.3
        self.marker.color.b = 0.3

        # marker position initial
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0.03
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0
        
        
    
    # 控制吸泵      
    def pub_pump(self, flag):
        if flag:
            """start the suction pump"""
            self.mc.set_basic_output(1, 0)
            self.mc.set_basic_output(2, 1)
        else:
            """stop suction pump"""
            self.mc.set_basic_output(1, 1)
            self.mc.set_basic_output(2, 0)
            time.sleep(1)
            self.mc.set_basic_output(2, 1)

    # Grasping motion
    def move(self, x, y, color):
        
        print(color)
        
        angles = [
            [0.61, 45.87, -92.37, -41.3, 89.56, 9.58],  # init to point
            [18.8, -7.91, -54.49, -23.02, 89.56, -14.76],
            [17.22, -5.27, -52.47, -25.75, 89.73, -0.26],
        ]

        coords = [
            [145.0, -65.5, 280.1, 178.99, 7.67, -179.9],  # 初始化点 init point
            [253.8, 236.8, 224.6, -170, 6.87, -77.91],  # A分拣区 A sorting area
            [35.9, 235.4, 211.8, -169.33, -9.27, 88.3],  # B分拣区  B sorting area
            [266.5, -219.7, 209.3, -170, -3.64, -94.62],  # C分拣区 C sorting area
            [32, -228.3, 201.6, -168.07, -7.17, -92.56],  # D分拣区 D sorting area
        ]
        print('real_x, real_y:', round(coords[0][0] + x, 2), round(coords[0][1] + y, 2))
        
        # publish marker
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = (coords[0][0]-x)/1000.0
        self.marker.pose.position.y = (coords[0][1]-y)/1000.0
        self.pub.publish(self.marker)

        # send coordinates to move mycobot
        self.mc.send_angles(angles[2], 50)
        time.sleep(3)
        
        self.mc.send_coords([coords[0][0] + x, coords[0][1] + y, 240, 178.99, -3.78, -62.9], 100, 1)
        time.sleep(2)
        self.mc.send_coords([coords[0][0] + x, coords[0][1] + y, 100.5, 178.99, -3.78, -62.9], 100, 1)
        time.sleep(2.5)
        
        # open pump
        if "dev" in self.robot_raspi:
            self.pub_pump(True)
        time.sleep(1.5)
        
        tmp = []
        while True:
            if not tmp: 
                tmp = self.mc.get_angles()    
            else:
                break
        time.sleep(0.5)
        
        # print(tmp)
        self.mc.send_angles([tmp[0], -0.71, -54.49, -23.02, 89.56, tmp[5]],25) # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        time.sleep(3)
        
        self.mc.send_coords(coords[color], 100, 1) # coords[1] 为A分拣区，coords[2] 为B分拣区, coords[3] 为C分拣区，coords[4] 为D分拣区
        time.sleep(6.5)
        
        # close pump
        if "dev" in self.robot_raspi:
            self.pub_pump(False)  
        time.sleep(6.5)
        
        # publish marker
        time.sleep(1)
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = coords[1][0]/1000.0
        self.marker.pose.position.y = coords[1][1]/1000.0
        self.pub.publish(self.marker)
        
        self.mc.send_angles(angles[0], 25)
        time.sleep(4.5)

    # decide whether grab cube
    def decide_move(self, x, y, color):

        print(x,y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5: # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x + 105, y + 130, color)

    # init mycobot
    def init_mycobot(self):
        if "dev" in self.robot_raspi:
            self.mc = MyCobot(self.robot_raspi, 115200)
        self.pub_pump(False)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 89.56, 9.58], 20)
        time.sleep(2.5)
        


    def run(self):
        global pump_y, pump_x
        self.init_mycobot()
        print('ok')
        num = sum_x = sum_y = 0 
        while cv2.waitKey(1) < 0:
            success, img = self.cap.read()
            if not success:
                print("It seems that the image cannot be acquired correctly.")
                break

            # transfrom the img to model of gray
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Detect ArUco marker.
            corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            # Determine the placement point of the QR code
            if ids == np.array([[1]]):
                self.color = 1
            elif ids == np.array([[2]]):
                self.color = 2
            elif ids == np.array([[3]]):
                self.color = 3
            elif ids == np.array([[4]]):
                self.color = 4

            if len(corners) > 0:
                if ids is not None:
                    # get informations of aruco
                    ret = cv2.aruco.estimatePoseSingleMarkers(
                        corners, 0.03, self.camera_matrix, self.dist_coeffs
                    )
                    # rvec:rotation offset,tvec:translation deviator
                    (rvec, tvec) = (ret[0], ret[1])
                    (rvec - tvec).any()
                    xyz = tvec[0, 0, :]
                    # calculate the coordinates of the aruco relative to the pump
                    xyz = [round(xyz[0]*1000+pump_y, 2), round(xyz[1]*1000+pump_x, 2), round(xyz[2]*1000, 2)]

                    # cv2.putText(img, str(xyz[:2]), (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    for i in range(rvec.shape[0]):
			# draw the aruco on img
                        cv2.aruco.drawDetectedMarkers(img, corners)
                        
                        if num < 40 :
                            sum_x += xyz[1]
                            sum_y += xyz[0]
                            num += 1
                        elif num ==40 :
                            self.decide_move(sum_x/40.0, sum_y/40.0, self.color)
                            num = sum_x = sum_y = 0

            cv2.imshow("encode_image", img)

if __name__ == "__main__":
    detect = Detect_marker()
    detect.run()

