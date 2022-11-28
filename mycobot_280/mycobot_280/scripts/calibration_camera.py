#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import os
import numpy as np
import cv2 as cv

import cv2
import matplotlib.pyplot as plt

# 生成一张630*890的全黑图片
# img = np.zeros((630,890,3),np.uint8)
# plt.imshow(img[:,:,::-1])

# while True:
# # plt.show()
#     cv2.imshow('img', img)

#     key = cv2.waitKey(0)
#     if key == ord('q'):
#         break
#     elif key == ord('s'):
#         cv2.imwrite('/home/h/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts/123.png', img)
#         print('saved')

# cv2.destroyAllWindows()

path = os.path.join(os.path.dirname(__file__), "3a4.bmp")
print(path)

frame = cv2.imread(path)
row, col, nc = frame.shape

width_of_roi = 90
# 这里是对全黑图片做处理，将图片以黑白间隔的形式zh
for j in range(row):
    data = frame[j]
    for i in range(col):
        f = int(i / width_of_roi) % 2 ^ int(j / width_of_roi) % 2
        if f:
            frame[j][i][0] = 255
            frame[j][i][1] = 255
            frame[j][i][2] = 255
cv2.imshow("", frame)
cv2.waitKey(0) & 0xFF == ord("q")
cv2.imwrite(os.path.join(os.path.dirname(__file__), "1245.jpg"), frame)


# import os
# import cv2
# import threading

# if_save = False
# # 设置摄像头编号（由于电脑型号不同，分配给USB摄像头的编号也可能不同，一般为0或1）
# cap_num = int(input("Input the camare number:"))
# # 设置所存储的图片名称，设置为1，即表示从1开始累加存储。如：1.jpg,2.jpg,3.jpg......
# name = int(input("Input start name, use number:"))

# cap = cv2.VideoCapture(cap_num)
# dir_path = os.path.dirname(__file__)

# def save():
#     global if_save
#     while True:
#         input("Input any to save a image:")
#         if_save = True

# # 开启线程进行摄像头拍摄
# t = threading.Thread(target=save)
# # 设置为异步运行
# t.setDaemon(True)
# t.start()

# while cv2.waitKey(1) != ord("q"):
#     _, frame = cap.read()
#     if if_save:
#         # 设置名称为当前路径下，否则会因为运行环境的原因使得存储位置发生变化
#         img_name = os.path.join(dir_path,str(name)+".jpg") 
#         # 存储图片
#         cv2.imwrite(img_name, frame)
#         print("Save {} successful.".format(img_name))
#         name += 1
#         if_save = False
#     cv2.imshow("", frame)


# import os
# import glob
# import numpy as np
# import cv2 as cv
# from pprint import pprint

# obj_points = []  # 3d点在现实世界的位置。
# img_points = []  # 2d点在图片中的位置。

# gray = None

# def calibration_camera(row, col, path=None, cap_num=None, saving=False):
#     """校准摄像头

#     参数说明:
#         row (int): 网格中的行数。
#         col (int): 网格中的列数。
#         path (string): 存放校准图片的位置。
#         cap_num (int): 表示摄像头的编号，一般0或1
#         saving (bool): 是否存放相机矩阵和失真系数(.npz).
#     """

#     # 终止准则/失效准则
#     criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#     # 准备物体点, 比如 (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
#     obj_p = np.zeros((row * col, 3), np.float32)
#     obj_p[:, :2] = np.mgrid[0:row, 0:col].T.reshape(-1, 2)
#     # 组用于存储来自所有图像的对象点和图像点。
 

#     def _find_grid(img):
#         # 使用函数外的参数
#         global gray, obj_points, img_points
#         # 将图片转换为灰色度图片
#         gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#         # 寻找棋盘的角落
#         ret, corners = cv.findChessboardCorners(gray, (row, col), None)
#         # 如果找到，则添加处理后的2d点和3d点
#         if ret == True:
#             obj_points.append(obj_p)
#             corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
#             img_points.append(corners)
#             # 在图片中绘制并展示所寻找到的角
#             cv.drawChessboardCorners(img, (row, col), corners2, ret)

#     # 要求必须选择使用图片校准或者摄像头实时捕获校准中的一种
#     if path and cap_num:
#         raise Exception("The parameter `path` and `cap_num` only need one.")
#     # 图片校准
#     if path:
#         # 获取当前路径中的所有图片
#         images = glob.glob(os.path.join(path, "*.jpg"))
#         pprint(images)
#         # 对获取的每张图片进行处理
#         for f_name in images:
#             # 读取图片
#             img = cv.imread(f_name)
#             _find_grid(img)
#             # 展示图片
#             cv.imshow("img", img)
#             # 图片展示等待0.5s
#             cv.waitKey(500)
#     # 摄像头实时捕获校准
#     if cap_num:
#         # 开启摄像头
#         cap = cv.VideoCapture(cap_num)
#         while True:
#             # 读取摄像头开启后的每帧图片
#             _, img = cap.read()
#             _find_grid(img)
#             cv.imshow("img", img)
#             cv.waitKey(500)
#             print(len(obj_points))
#             if len(obj_points) > 14:
#                 break
#     # 销毁展示窗口
#     cv.destroyAllWindows()
#     # 通过计算获取的3d点和2d点得出相机矩阵和失真系数
#     ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
#         obj_points, img_points, gray.shape[::-1], None, None
#     )
#     print("ret: {}".format(ret))
#     print("matrix:")
#     pprint(mtx)
#     print("distortion: {}".format(dist))
#     # 决定是否存储所计算出的参数
#     if saving:
#         np.savez(os.path.join(os.path.dirname(__file__), "camera_mtx_dist.npz"), mtx=mtx, dist=dist)

#     mean_error = 0
#     for i in range(len(obj_points)):
#         img_points_2, _ = cv.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
#         error = cv.norm(img_points[i], img_points_2, cv.NORM_L2) / len(img_points_2)
#         mean_error += error
#     print("total error: {}".format(mean_error / len(obj_points)))

#     return mtx, dist

# if __name__ == "__main__":
#     path = os.path.dirname(__file__)
#     mtx, dist = calibration_camera(8, 6, path, saving=True)
#     # 设置是否需要测试计算出的参数
#     if_test = input("If testing the result (default: no), [yes/no]:")
#     if if_test not in ["y", "Y", "yes", "Yes"]:
#         exit(0)

#     cap_num = int(input("Input camera number:"))
#     cap = cv.VideoCapture(cap_num)
#     while cv.waitKey(1) != ord("q"):
#         _, img = cap.read()
#         h, w = img.shape[:2]
#         # 相机校准
#         dst = cv.undistort(img, mtx, dist)
#         cv.imshow("", dst)