#encoding: UTF-8
import cv2
import numpy as np
from pymycobot.mycobot import MyCobot
from pymycobot import PI_BAUD, PI_PORT
import RPi.GPIO as GPIO
import time



# y轴偏移量
pump_y = -55
# x轴偏移量
pump_x = 15

class Detect_marker():
    def __init__(self):
        #initialize MyCobot
        self.mc = MyCobot(PI_PORT, PI_BAUD) 
        GPIO.setmode(GPIO.BCM)
        # GPIO.setup(20, GPIO.OUT)
        GPIO.setup(21, GPIO.OUT)
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        # Creating a Camera Object
        cap_num = 0
        self.cap = cv2.VideoCapture(cap_num)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
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
    
    # 控制吸泵      
    def pub_pump(self, flag):
        if flag:
           # GPIO.output(20, 0)
            GPIO.output(21, 0)
        else:
           # GPIO.output(20, 1)
            GPIO.output(21, 1)

    # Grasping motion
    def move(self, x, y):

        coords = [
            [135.0, -65.5, 280.1, 178.99, 5.38, -179.9], # 初始化点 init point
            [132.2, -136.9, 200.8, -178.24, -3.72, -107.17],  # D分拣区 D sorting area
            [238.8, -124.1, 204.3, -169.69, -5.52, -96.52], # C分拣区 C sorting area
            [115.8, 177.3, 210.6, 178.06, -0.92, -6.11], # A分拣区 A sorting area
            [-6.9, 173.2, 201.5, 179.93, 0.63, 33.83], # B分拣区  B sorting area
        ]

        # send coordinates to move mycobot
        self.mc.send_coords(coords[0], 30, 1)
        time.sleep(2)
        self.mc.send_coords([coords[0][0]+x, coords[0][1]+y, 240, 178.99, 5.38, -179.9], 20, 0)
        time.sleep(2)
        self.mc.send_coords([coords[0][0]+x, coords[0][1]+y, 200, 178.99, 5.38, -179.9], 20, 0)
        time.sleep(2)
        self.mc.send_coords([coords[0][0]+x, coords[0][1]+y, 105, 178.99, 5.38, -179.9], 20, 0)
        time.sleep(3.5)
        self.pub_pump(True)
        self.mc.send_coords(coords[0], 30, 1)
        time.sleep(4)
        self.mc.send_coords(coords[1], 30, 1)
        time.sleep(4)
        self.pub_pump(False)  
        time.sleep(5)
        self.mc.send_coords(coords[0], 30, 1)
        time.sleep(2)

    # decide whether grab cube
    def decide_move(self, x, y):

        print(x,y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5: # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x-10, y+145)

    # init mycobot
    def init_mycobot(self):
        self.pub_pump(False)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 20)
        # self.mc.send_coords([135.0, -65.5, 280.1, 178.99, 5.38, -179.9], 20, 1)
        time.sleep(4.5)
        


    def run(self):
        global pump_y, pump_x
        self.init_mycobot()
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

                    cv2.putText(img, 'coords' + str(xyz), (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    for i in range(rvec.shape[0]):
			# draw the aruco on img
                        cv2.aruco.drawDetectedMarkers(img, corners)
                        
                        if num < 40 :
                            sum_x += xyz[1]
                            sum_y += xyz[0]
                            num += 1
                        elif num ==40 :
                            self.decide_move(sum_x/40.0, sum_y/40.0)
                            num = sum_x = sum_y = 0

            cv2.imshow("encode_image", img)

if __name__ == "__main__":
    detect = Detect_marker()
    detect.run()

