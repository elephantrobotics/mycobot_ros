import cv2
import numpy as np
import time
import os,sys
import math

from pymycobot.mycobot import MyCobot


IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"
# Adaptive seeed


class Object_detect():

    def __init__(self, camera_x = 162, camera_y = 15):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # declare mycobot280
        self.mc = None

        # 移动角度
        self.move_angles = [
            [0.61, 45.87, -92.37, -41.3, 2.02, 9.58],  # init the point
            [18.8, -7.91, -54.49, -23.02, -0.79, -14.76],  # point to grab
        ]

        # 移动坐标
        self.move_coords = [
            [132.2, -136.9, 200.8, -178.24, -3.72, -107.17],  # D Sorting area
            [238.8, -124.1, 204.3, -169.69, -5.52, -96.52], # C Sorting area
            [115.8, 177.3, 210.6, 178.06, -0.92, -6.11], # A Sorting area
            [-6.9, 173.2, 201.5, 179.93, 0.63, 33.83], # B Sorting area
        ]
        
        # which robot: USB* is m5; ACM* is wio; AMA* is raspi
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        self.robot_raspi = os.popen("ls /dev/ttyAMA*").readline()[:-1]
        self.robot_jes = os.popen("ls /dev/ttyTHS1").readline()[:-1]
        self.raspi = False
        if "dev" in self.robot_m5:
            self.Pin = [2, 5]
        elif "dev" in self.robot_wio:
            # self.Pin = [20, 21]
            self.Pin = [2, 5]

            for i in self.move_coords:
                i[2] -= 20
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(20, GPIO.OUT)
            GPIO.setup(21, GPIO.OUT)
            GPIO.output(20, 1)
            GPIO.output(21, 1)
            self.raspi = True
        if self.raspi:
            self.gpio_status(False)
            
            
        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        # set color HSV
        self.HSV = {
            # "yellow": [np.array([11, 85, 70]), np.array([59, 255, 245])],
            "yellow": [np.array([22, 93, 0]), np.array([45, 255, 245])],
            "red": [np.array([0, 43, 46]), np.array([8, 255, 255])],
            "green": [np.array([35, 43, 35]), np.array([90, 255, 255])],
            "blue": [np.array([100, 43, 46]), np.array([124, 255, 255])],
            "cyan": [np.array([78, 43, 46]), np.array([99, 255, 255])],
        }
 
        # use to calculate coord between cube and mycobot
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the mycobot
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the mycobot
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = 0
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # 初始化背景减法器
        self.mog =cv2.bgsegm.createBackgroundSubtractorMOG() 
    
    # pump_control pi
    def gpio_status(self, flag):
        if flag:
            # self.GPIO.output(20, 0)
            self.GPIO.output(21, 0)
        else:
            # self.GPIO.output(20, 1)
            self.GPIO.output(21, 1)
    
    # 开启吸泵 m5
    def pump_on(self):
        # 让2号位工作
        # self.mc.set_basic_output(2, 0)
        # 让5号位工作
        self.mc.set_basic_output(5, 0)

    # 停止吸泵 m5
    def pump_off(self):
        # 让2号位停止工作
        # self.mc.set_basic_output(2, 1)
        # 让5号位停止工作
        self.mc.set_basic_output(5, 1)

    # Grasping motion
    def move(self, x, y, color):
        # send Angle to move mycobot280
        print(color)
        self.mc.send_angles(self.move_angles[1], 25)
        time.sleep(3)

        # send coordinates to move mycobot
        self.mc.send_coords([x, y,  190.6, 179.87, -3.78, -62.75], 25, 1) # usb :rx,ry,rz -173.3, -5.48, -57.9
        time.sleep(3)
        
        # self.mc.send_coords([x, y, 150, 179.87, -3.78, -62.75], 25, 0)
        # time.sleep(3)

        self.mc.send_coords([x, y, 65.5, 179.87, -3.78, -62.75], 25, 0)
        time.sleep(4)

        # open pump
        if "dev" in self.robot_m5 or "dev" in self.robot_wio:
            self.pump_on()
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            self.gpio_status(True)
        time.sleep(1.5)

        tmp = []
        while True:
            if not tmp: 
                tmp = self.mc.get_angles()    
            else:
                break
        time.sleep(0.5)
        
        # print(tmp)
        self.mc.send_angles([tmp[0], -0.71, -54.49, -23.02, -0.79, tmp[5]],25) # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        time.sleep(2.5)

        self.mc.send_coords(self.move_coords[color], 25, 1)
 
        time.sleep(3)
       
        # close pump
 
        if "dev" in self.robot_m5 or "dev" in self.robot_wio:
            self.pump_off()
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            self.gpio_status(False)
        time.sleep(5)

        self.mc.send_angles(self.move_angles[0], 25)
        time.sleep(4.5)

    # decide whether grab cube
    def decide_move(self, x, y, color):
        print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x, y, color)

    # init mycobot280
    def run(self):
     
        if "dev" in self.robot_wio :
            self.mc = MyCobot(self.robot_wio, 115200) 
        elif "dev" in self.robot_m5:
            self.mc = MyCobot(self.robot_m5, 115200) 
        elif "dev" in self.robot_raspi:
            self.mc = MyCobot(self.robot_raspi, 1000000)
        if not self.raspi:
            self.pub_pump(False, self.Pin)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 20)
        time.sleep(4.5)

    # draw aruco
    def draw_marker(self, img, x, y):
        # draw rectangle on img
        cv2.rectangle( 
            img,
            (x - 20, y - 20),
            (x + 20, y + 20),
            (0, 255, 0),
            thickness=2,
            lineType=cv2.FONT_HERSHEY_COMPLEX,
        )
        # add text on rectangle
        cv2.putText(img, "({},{})".format(x, y), (x, y),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (243, 0, 0), 2,)

    # get points of two aruco
    def get_calculate_params(self, img):
        # Convert the image to a gray image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect ArUco marker.
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        """
        Two Arucos must be present in the picture and in the same order.
        There are two Arucos in the Corners, and each aruco contains the pixels of its four corners.
        Determine the center of the aruco by the four corners of the aruco.
        """
        if len(corners) > 0:
            if ids is not None:
                if len(corners) <= 1 or ids[0] == 1:
                    return None
                x1 = x2 = y1 = y2 = 0
                point_11, point_21, point_31, point_41 = corners[0][0]
                x1, y1 = int((point_11[0] + point_21[0] + point_31[0] + point_41[0]) / 4.0), int(
                    (point_11[1] + point_21[1] + point_31[1] + point_41[1]) / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int((point_1[0] + point_2[0] + point_3[0] + point_4[0]) / 4.0), int(
                    (point_1[1] + point_2[1] + point_3[1] + point_4[1]) / 4.0)
                return x1, x2, y1, y2
        return None

    # set camera clipping parameters
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)
        print(self.x1, self.y1, self.x2, self.y2)

    # set parameters to calculate the coords between cube and mycobot280
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0/ratio

    # calculate the coords between cube and mycobot280
    def get_position(self, x, y):
        return ((y - self.c_y)*self.ratio + self.camera_x), ((x - self.c_x)*self.ratio + self.camera_y)

    """
    Calibrate the camera according to the calibration parameters.
    Enlarge the video pixel by 1.5 times, which means enlarge the video size by 1.5 times.
    If two ARuco values have been calculated, clip the video.
    """
    def transform_frame(self, frame):
        # enlarge the image by 1.5 times
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0), fx=fx, fy=fy,
                           interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            # the cutting ratio here is adjusted according to the actual situation
            frame = frame[int(self.y2*0.78):int(self.y1*1.1),
                          int(self.x1*0.88):int(self.x2*1.06)]
        return frame
    
    # 检测物体的形状
    def shape_detect(self,img):
        x = 0
        y = 0
        Alpha = 65.6
        Gamma=-8191.5
        cal = cv2.addWeighted(img, Alpha,img, 0, Gamma)
        gray = cv2.cvtColor(cal, cv2.COLOR_BGR2GRAY)
    
        # 转换为灰度图片
        #ray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # a etching operation on a picture to remove edge roughness
        erosion = cv2.erode(gray, np.ones((2, 2), np.uint8), iterations=2)

        # the image for expansion operation, its role is to deepen the color depth in the picture
        dilation = cv2.dilate(erosion, np.ones(
            (1, 1), np.uint8), iterations=2)


        # 设定灰度图的阈值 175, 255
        _, threshold = cv2.threshold(dilation, 175, 255, cv2.THRESH_BINARY)
        # 边缘检测
        edges = cv2.Canny(threshold,50,100)
        # 检测物体边框
        contours,_ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        


        if len(contours)>0:
            for cnt in contours:
                # if 6000>cv2.contourArea(cnt) and cv2.contourArea(cnt)>4500:
                if cv2.contourArea(cnt)>5500:
                    objectType = None
                    peri = cv2.arcLength(cnt,True)
                    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                    objCor = len(approx)
                    x, y, w, h = cv2.boundingRect(approx)

                    boxes = [
                        box
                        for box in [cv2.boundingRect(c) for c in contours]
                        if min(img.shape[0], img.shape[1]) / 10
                        < min(box[2], box[3])
                        < min(img.shape[0], img.shape[1]) / 1
                    ]
                    if boxes:
                        for box in boxes:
                            x, y, w, h = box
                        # find the largest object that fits the requirements
                        c = max(contours, key=cv2.contourArea)
                        rect = cv2.minAreaRect(c)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        cv2.drawContours(img, [box], 0, (153, 153, 0), 2)
                        x = int(rect[0][0])
                        y = int(rect[0][1])

                    if objCor==3:
                        objectType = "Triangle(三角形)"
                        cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                        self.color = 3
                    elif objCor==4:
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        _W = math.sqrt(math.pow((box[0][0] - box[1][0]), 2) + math.pow((box[0][1] - box[1][1]), 2))
                        _H = math.sqrt(math.pow((box[0][0] - box[3][0]), 2) + math.pow((box[0][1] - box[3][1]), 2))
                        aspRatio = _W/float(_H)
                        if 0.98 < aspRatio < 1.03:
                            objectType = "Square(正方形)"
                            cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                            self.color=1
                        else:
                            objectType = "Rectangle(长方形)"
                            cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                            self.color=2
                    elif objCor>=5:
                        objectType = "Circle(圆形)"
                        self.color=0
                        cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                    else:
                        pass
                    print(objectType)

        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None

        
if __name__ == "__main__":

    # open the camera
    cap_num = 0
    # cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
    cap = cv2.VideoCapture(cap_num)
    cap.set(3, 640)
    cap.set(4, 480)
    if not cap.isOpened():
        cap.open()
    # init a class of Object_detect
    detect = Object_detect()
    # init mycobot280
    detect.run()

    _init_ = 20  
    init_num = 0
    nparams = 0
    num = 0
    real_sx = real_sy = 0
    while cv2.waitKey(1) < 0:
       # read camera
        _, frame = cap.read()
        # deal img
        frame = detect.transform_frame(frame)
        if _init_ > 0:
            _init_ -= 1
            continue

        # calculate the parameters of camera clipping
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                init_num += 1
                continue
        elif init_num == 20:
            detect.set_cut_params(
                (detect.sum_x1)/20.0,
                (detect.sum_y1)/20.0,
                (detect.sum_x2)/20.0,
                (detect.sum_y2)/20.0,
            )
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and mycobot280
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                nparams += 1
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and mycobot280
            detect.set_params(
                (detect.sum_x1+detect.sum_x2)/20.0,
                (detect.sum_y1+detect.sum_y2)/20.0,
                abs(detect.sum_x1-detect.sum_x2)/10.0 +
                abs(detect.sum_y1-detect.sum_y2)/10.0
            )
            print("ok")
            continue

        # get detect result
        # detect_result = detect.color_detect(frame)
        # print('调用检测')
        detect_result = detect.shape_detect(frame)
        # print("完成检测")
        if detect_result is None:
            cv2.imshow("figure", frame)
            continue
        else:
            x, y = detect_result
            # calculate real coord between cube and mycobot280
            real_x, real_y = detect.get_position(x, y)
            if num == 20:
                
                detect.decide_move(real_sx/20.0, real_sy/20.0, detect.color)
                num = real_sx = real_sy = 0

            else:
                num += 1
                real_sy += real_y
                real_sx += real_x

        cv2.imshow("figure", frame)

        # close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            sys.exit()
