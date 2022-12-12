# encoding:utf-8
#!/usr/bin/env python2

from multiprocessing import Process, Pipe

from cgi import parse
from difflib import restore
# import queue
from sys import path
from tokenize import Pointfloat
from turtle import color
# from typing_extensions import Self
import cv2
import numpy as np
import time
import json
import os
import rospy
from visualization_msgs.msg import Marker
from PIL import Image
from threading import Thread
import tkFileDialog as filedialog
import Tkinter as tk
from moving_utils import Movement

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"  # Adaptive seeed


class Object_detect(Movement):

    def __init__(self, camera_x=150, camera_y=-10):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # get path of file
        dir_path = os.path.dirname(__file__)

        # 移动角度
        self.move_angles = [
            [-26.11, -6.94, -55.01, -24.16, 0, 15],  # init the point
            [-1.14, 0.63, -87.8, 9.05, -3.07, 15],  # point to grab
            [17.4, -10.1, -87.27, 5.8, -2.02, 15],  # point to grab
        ]

        # 移动坐标
        self.move_coords = [
            [120.1, -141.6, 240.9, -173.34, -8.15, -110.11],  # above the red bucket
            # above the yello bucket
            #[208.2, -127.8, 260.9, -157.51, -17.5, -71.18],
            [205.6, -130.5, 263.0, -150.99, -0.07, -107.35],  # above the green bucket
            [-20.0, 176.7, 242.6, -166.66, -9.44, -52.47],  # above the gray bucket
            [104.9, 176.7, 242.6, -166.66, -9.44, -52.47],  # above the blue bucket
            [126.6, -118.1, 305.0, -157.57, -13.72, -75.3],  
        ]

        # 判断连接设备:ttyUSB*为M5，ttyACM*为seeed
        
        self.raspi = False
        import RPi.GPIO as GPIO
        self.GPIO = GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)
        GPIO.setup(21, GPIO.OUT)

        self.gpio_status(False)

        self.Pin = [2, 5]
        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        # load model of img recognition
        # self.model_path = os.path.join(dir_path, "frozen_inference_graph.pb")
        # self.pbtxt_path = os.path.join(dir_path, "graph.pbtxt")
        # self.label_path = os.path.join(dir_path, "labels.json")
        # # load class labels
        # self.labels = json.load(open(self.label_path))

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

        # if IS_CV_4:
        #     self.net = cv2.dnn.readNetFromTensorflow(self.model_path, self.pbtxt_path)
        # else:
        #     print('Load tensorflow model need the version of opencv is 4.')
        #     exit(0)
        # init a node and a publisher
        rospy.init_node("marker", anonymous=True)
        self.pub = rospy.Publisher('/cube', Marker, queue_size=1)
        # init a Marker
        self.marker = Marker()
        self.marker.header.frame_id = "/joint1"
        self.marker.ns = "cube"
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.04
        self.marker.scale.y = 0.04
        self.marker.scale.z = 0.04
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.color.r = 1.0

        # marker position initial
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0.03
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0

        self.cache_x = self.cache_y = 0

    # publish marker

    def pub_marker(self, x, y, z=0.03):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.color.g = self.color
        self.pub.publish(self.marker)

    def gpio_status(self, flag):
        if flag:
            self.GPIO.output(20, 0)
            self.GPIO.output(21, 0)
        else:
            self.GPIO.output(20, 1)
            self.GPIO.output(21, 1)

    # Grasping motion
    def move(self, x, y, color):
        # send Angle to move mycobot
        self.pub_angles(self.move_angles[0], 20)
        time.sleep(1.5)
        self.pub_angles(self.move_angles[1], 20)
        time.sleep(1.5)
        self.pub_angles(self.move_angles[2], 20)
        time.sleep(1.5)
        # send coordinates to move mycobot
        self.pub_coords([x, y, 165, -178.9, -1.57, -66], 20, 1)
        time.sleep(1.5)
        # 根据不同底板机械臂，调整吸泵高度
        self.pub_coords([x, y, 90, -178.9, -1.57, -66], 25, 1)
        time.sleep(1.5)
        # open pump
        self.gpio_status(True)
        
        time.sleep(0.5)
        self.pub_angles(self.move_angles[2], 20)
        time.sleep(3)
        self.pub_marker(self.move_coords[2][0] / 1000.0,
                        self.move_coords[2][1] / 1000.0,
                        self.move_coords[2][2] / 1000.0)

        self.pub_angles(self.move_angles[1], 20)
        time.sleep(1.5)
        self.pub_marker(self.move_coords[3][0] / 1000.0,
                        self.move_coords[3][1] / 1000.0,
                        self.move_coords[3][2] / 1000.0)

        self.pub_angles(self.move_angles[0], 20)
        time.sleep(1.5)
        self.pub_marker(self.move_coords[4][0] / 1000.0,
                        self.move_coords[4][1] / 1000.0,
                        self.move_coords[4][2] / 1000.0)

        print(self.move_coords[color])
        self.pub_coords(self.move_coords[color], 20, 1)
        self.pub_marker(self.move_coords[color][0] / 1000.0,
                        self.move_coords[color][1] / 1000.0,
                        self.move_coords[color][2] / 1000.0)
        time.sleep(4)
        # close pump
        self.gpio_status(False)
        
        time.sleep(1)
        self.pub_marker(self.move_coords[color][0] / 1000.0 + 0.04,
                        self.move_coords[color][1] / 1000.0 - 0.02)
        self.pub_angles(self.move_angles[0], 20)
        time.sleep(3)

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
            print(x, y)
            self.move(x, y, color)

    # init mycobot
    def run(self):
        for _ in range(5):
            self.pub_angles([-26.11, -6.94, -55.01, -24.16, 0, -15], 20)
            print(_)
            time.sleep(0.5)

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
        cv2.putText(
            img,
            "({},{})".format(x, y),
            (x, y),
            cv2.FONT_HERSHEY_COMPLEX_SMALL,
            1,
            (243, 0, 0),
            2,
        )

    # get points of two aruco
    def get_calculate_params(self, img):
        # Convert the image to a gray image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect ArUco marker.
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
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
                x1, y1 = int(
                    (point_11[0] + point_21[0] + point_31[0] + point_41[0]) /
                    4.0), int(
                        (point_11[1] + point_21[1] + point_31[1] + point_41[1])
                        / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int(
                    (point_1[0] + point_2[0] + point_3[0] + point_4[0]) /
                    4.0), int(
                        (point_1[1] + point_2[1] + point_3[1] + point_4[1]) /
                        4.0)
                return x1, x2, y1, y2
        return None

    # set camera clipping parameters
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)
        print(self.x1, self.y1, self.x2, self.y2)

    # set parameters to calculate the coords between cube and mycobot
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0 / ratio

    # calculate the coords between cube and mycobot
    def get_position(self, x, y):
        return ((y - self.c_y) * self.ratio +
                self.camera_x), ((x - self.c_x) * self.ratio + self.camera_y)

    """
    Calibrate the camera according to the calibration parameters.
    Enlarge the video pixel by 1.5 times, which means enlarge the video size by 1.5 times.
    If two ARuco values have been calculated, clip the video.
    """

    def transform_frame(self, frame):
        # enlarge the image by 1.5 times
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0),
                           fx=fx,
                           fy=fy,
                           interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            # the cutting ratio here is adjusted according to the actual situation
            frame = frame[int(self.y2 * 0.2):int(self.y1 * 1.15),
                          int(self.x1 * 0.7):int(self.x2 * 1.15)]
        return frame

    # according the class_id to get object name
    def id_class_name(self, class_id):
        for key, value in self.labels.items():
            if class_id == int(key):
                return value

    # detect object
    def obj_detect(self, img, goal, kp_img, desc_img, kp_list, desc_list, connection):
        i = 0
        MIN_MATCH_COUNT = 5
        # sift = cv2.xfeatures2d.SIFT_create()

        # find the keypoints and descriptors with SIFT
        # kp = []
        # des = []
        kp = kp_list
        des = desc_list

        # for i in goal:
        #     kp0, des0 = sift.detectAndCompute(i, None)
        #     kp.append(kp0)
        #     des.append(des0)

        # kp1, des1 = sift.detectAndCompute(goal, None)
        # kp2, des2 = sift.detectAndCompute(img, None)
        kp2, des2 = kp_img, desc_img

        # FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)  # or pass empty dictionary
        flann = cv2.FlannBasedMatcher(index_params, search_params)

        x, y = 0, 0
        try:
            for i in range(len(des)):
                matches = flann.knnMatch(des[i], des2, k=2)
                # store all the good matches as per Lowe's ratio test.  根据Lowe比率测试存储所有良好匹配项。
                good = []
                for m, n in matches:
                    if m.distance < 0.7 * n.distance:
                        good.append(m)

                # When there are enough robust matching point pairs 当有足够的健壮匹配点对（至少个MIN_MATCH_COUNT）时
                if len(good) > MIN_MATCH_COUNT:

                    # extract corresponding point pairs from matching 从匹配中提取出对应点对
                    # query index of small objects, training index of scenarios 小对象的查询索引，场景的训练索引
                    src_pts = np.float32([kp[i][m.queryIdx].pt
                                          for m in good]).reshape(-1, 1, 2)
                    dst_pts = np.float32([kp2[m.trainIdx].pt
                                          for m in good]).reshape(-1, 1, 2)

                    # Using matching points to find homography matrix in cv2.ransac 利用匹配点找到CV2.RANSAC中的单应矩阵
                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,
                                                 5.0)
                    matchesMask = mask.ravel().tolist()
                    # Calculate the distortion of image, that is the corresponding position in frame 计算图1的畸变，也就是在图2中的对应的位置
                    h, w, d = goal[i].shape
                    pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1],
                                      [w - 1, 0]]).reshape(-1, 1, 2)
                    dst = cv2.perspectiveTransform(pts, M)
                    coord = (dst[0][0] + dst[1][0] + dst[2][0] +
                              dst[3][0]) / 4.0
                    connection.send((DRAW_COORDS, coord))
                    # cv2.putText(img, "{}".format(coord), (50, 60),
                    #         fontFace=None, fontScale=1,
                    #         color=(0, 255, 0), lineType=1)
                    print(format(dst[0][0][0]))
                    x = (dst[0][0][0] + dst[1][0][0] + dst[2][0][0] +
                         dst[3][0][0]) / 4.0
                    y = (dst[0][0][1] + dst[1][0][1] + dst[2][0][1] +
                         dst[3][0][1]) / 4.0

                    # bound box  绘制边框
                    # img = cv2.polylines(img, [np.int32(dst)], True, 244, 3, cv2.LINE_AA)
                    connection.send((DRAW_RECT, dst))
                    # cv2.polylines(mixture, [np.int32(dst)], True, (0, 255, 0), 2, cv2.LINE_AA)
        except Exception as e:
            pass

        # else:
        #     if(len(good) < MIN_MATCH_COUNT):

        #         i += 1
        #         if(i % 10 == 0):
        #             print("Not enough matches are found - %d/%d" %
        #                   (len(good), MIN_MATCH_COUNT))

        #     matchesMask = None
        if x + y > 0:
            return x, y
        else:
            return None

# The path to save the image folder
def parse_folder(folder):
    restore = []
    path = '/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_ai/myCobot_280' + folder
    for i, j, k in os.walk(path):
        for l in k:
            restore.append(cv2.imread(folder + '/{}'.format(l)))
    return restore


def compute_keypoints_and_descriptors(sift, images_lists):
    kp_list = []
    desc_list = []
    for images in images_lists:
        kp_tmp = []
        desc_tmp = []
        for img in images:
            kp, desc = sift.detectAndCompute(img, None)
            kp_tmp.append(kp)
            desc_tmp.append(desc)
        kp_list.append(kp_tmp)
        desc_list.append(desc_tmp)

    return kp_list, desc_list


GET_FRAME = 1
STOP_PROCESSING = 2
DRAW_COORDS = 3
DRAW_RECT = 4
CLEAR_DRAW = 5
CROP_FRAME = 6


def get_frame(connection):
    connection.send(GET_FRAME)
    frame = connection.recv()
    return frame


def process_transform_frame(frame, x1, y1, x2, y2):
    # enlarge the image by 1.5 times
    fx = 1.5
    fy = 1.5
    frame = cv2.resize(frame, (0, 0),
                        fx=fx,
                        fy=fy,
                        interpolation=cv2.INTER_CUBIC)
#    if x1 != x2:
        # the cutting ratio here is adjusted according to the actual situation
#        frame = frame[int(y2 * 0.2):int(y1 * 1.15),
#                        int(x1 * 0.7):int(x2 * 1.15)]
    return frame


def process_display_frame(connection):
    cap_num = 0
    coord = None
    dst = None
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0
    cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
    if not cap.isOpened():
        cap.open()
    while cv2.waitKey(1) < 0:
        _, frame = cap.read()
        frame = process_transform_frame(frame, x1, y1, x2, y2)
        if connection.poll():
            request = connection.recv()
            if request == GET_FRAME:
                connection.send(frame)
            elif request == CLEAR_DRAW:
                coord = None
                dst = None
            elif type(request) is tuple:
                if request[0] == DRAW_COORDS:
                    coord = request[1]
                elif request[0] == DRAW_RECT:
                    dst = request[1]
                elif request[0] == CROP_FRAME:
                    x1 = request[1]
                    y1 = request[2]
                    x2 = request[3]
                    y2 = request[4]

        if not coord is None:
            cv2.putText(frame, "{}".format(coord), (50, 60), fontFace=None,
                    fontScale=1, color=(0, 255, 0), lineType=1)
        if not dst is None:
            frame = cv2.polylines(frame, [np.int32(dst)], True, 244, 3, cv2.LINE_AA)
        cv2.imshow("figure", frame)
        time.sleep(0.04)
    connection.send(STOP_PROCESSING)


def run():
    parent_conn, child_conn = Pipe()

    child = Process(target=process_display_frame, args=(child_conn,))
    child.start()

    # Object_detect().take_photo()
    # Object_detect().cut_photo()
    # goal = Object_detect().distinguist()
    
    res_queue = [[], [], [], []]
    res_queue[0] = parse_folder('res/red')
    res_queue[1] = parse_folder('res/green')
    res_queue[2] = parse_folder('res/gray')
    res_queue[3] = parse_folder('res/blue')

    # res_queue = []
    # res_queue.extend(parse_folder('res/red'))
    # res_queue.extend(parse_folder('res/green'))
    # res_queue.extend(parse_folder('res/gray'))
    # res_queue.extend(parse_folder('res/blue'))

    sift = cv2.xfeatures2d.SIFT_create()
    kp_list, desc_list = compute_keypoints_and_descriptors(sift, res_queue)

    # init a class of Object_detect
    detect = Object_detect()
    # init mycobot
    detect.run()

    # _init_ = 20  #
    init_num = 0
    nparams = 0
    # num = 0
    # real_sx = real_sy = 0
    while True:
        start_time = time.time()
        if parent_conn.poll():
            data = parent_conn.recv()
            if data == STOP_PROCESSING:
                break
        # read camera
        frame = get_frame(parent_conn)
        # deal img
        #frame = detect.transform_frame(frame)

        # if _init_ > 0:
        #     _init_ -= 1
        #     continue
        # calculate the parameters of camera clipping
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                # cv2.imshow("figure", frame)
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
                (detect.sum_x1) / 20.0,
                (detect.sum_y1) / 20.0,
                (detect.sum_x2) / 20.0,
                (detect.sum_y2) / 20.0,
            )
            parent_conn.send((CROP_FRAME,
                (detect.sum_x1) / 20.0,
                (detect.sum_y1) / 20.0,
                (detect.sum_x2) / 20.0,
                (detect.sum_y2) / 20.0))
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and mycobot
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                # cv2.imshow("figure", frame)
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
                print ("ok")
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and mycobot
            detect.set_params((detect.sum_x1 + detect.sum_x2) / 20.0,
                              (detect.sum_y1 + detect.sum_y2) / 20.0,
                              abs(detect.sum_x1 - detect.sum_x2) / 10.0 +
                              abs(detect.sum_y1 - detect.sum_y2) / 10.0)
            print("ok")
            continue

        # get detect result

        kp_img, desc_img = sift.detectAndCompute(frame, None)

        frame = get_frame(parent_conn)
        for i, v in enumerate(res_queue):
            # HACK: to update frame every time
            detect_result = detect.obj_detect(frame, v, kp_img, desc_img, kp_list[i], desc_list[i], parent_conn)
            if detect_result:
                x, y = detect_result
                # calculate real coord between cube and mycobot
                real_x, real_y = detect.get_position(x, y)
                detect.color = i
                detect.pub_marker(real_x / 1000.0, real_y / 1000.0)
                detect.decide_move(real_x, real_y, detect.color)
                # if num == 5:
                #     detect.color = i
                #     detect.pub_marker(real_sx / 5.0 / 1000.0,
                #                       real_sy / 5.0 / 1000.0)
                #     detect.decide_move(real_sx / 5.0, real_sy / 5.0,
                #                        detect.color)
                #     num = real_sx = real_sy = 0
                # else:
                #     num += 1
                #     real_sy += real_y
                #     real_sx += real_x
                parent_conn.send(CLEAR_DRAW)

        # cv2.imshow("figure", frame)
        time.sleep(0.05)
        end_time = time.time()
        print("loop_time = ", end_time - start_time)

    child.join()


if __name__ == "__main__":
    run()
    # Object_detect().take_photo()
    # Object_detect().cut_photo()
