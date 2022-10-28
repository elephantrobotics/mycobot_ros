#!/usr/bin/python
# -*- coding: UTF-8 -*-
import numpy as np
import time
import cv2
import math
from pymycobot.mycobot import MyCobot
import time
import threading


mc = MyCobot('/dev/ttyUSB0', 115200)
mc.set_tool_reference([-50,0,0,0,0,0])
mc.set_end_type(1)

init_angles = [0, 0.52, -85.69, 0.0, 89.82, 0.08]
start_angles = [18.64, 0.52, -85.69, 0.0, 89.82, 0.08]
end_angles = [-18.56, 0.52, -85.69, 0.0, 89.82, 0.08]

ang = []
cood = []

def rotationVectorToEulerAngles(rvec):
    R = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(rvec, R)
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:  # 偏航，俯仰，滚动
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    # 偏航，俯仰，滚动换成角度
    rx = x * 180.0 / 3.141592653589793
    ry = y * 180.0 / 3.141592653589793
    rz = z * 180.0 / 3.141592653589793
    return rx, ry, rz

# 320 camera matrix parameters
mtx = np.array([[611.37813149,   0. ,350.02379708], [0.  , 612.14706662, 237.3623127], [ 0, 0, 1]])
# 320 camera distortion factor
dist = np.array(([[0.04106882, -0.11101182, -0.00039303,  0.00539495,  0.08251745]]))

cap = cv2.VideoCapture(0)
cap.set(5, 10)
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)


def wait_time(t):
    global ang
    for i in range(t*10+1):
        time.sleep(0.1)
        ang1 = mc.get_angles()
        ang.append(ang1)
        coord1 = mc.get_coords()
        cood.append(coord1)


def move():
    try:
        for _ in range(50):
            mc.send_angles(start_angles, 5)
            wait_time(6)
            mc.send_angles(end_angles, 5)
            wait_time(6)
    except Exception as e:
        print(e)

def target():
    i = 0

    while True:
    # while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        print('-->',cap.get(cv2.CAP_PROP_FPS))
        h1, w1 = frame.shape[:2]
        # print(h1, w1)
        # 读取摄像头画面
        # 纠正畸变
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h1, w1), 0, (h1, w1))
        dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        x, y, w1, h1 = roi
        # dst1 = dst1[y:y + h1, x:x + w1]
        frame = dst1
        # print(newcameramtx)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        '''
        detectMarkers(...)
            detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
            mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''

        #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    #    如果找不打id
        if ids is not None:

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.035, mtx, dist)
            # 估计每个标记的姿态并返回nt(值rvet和tvec ---不同
            # from camera coeficcients
            (rvec-tvec).any()# get rid of that nasty numpy value array error
            for i in range(rvec.shape[0]):
                cv2.aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
                cv2.aruco.drawDetectedMarkers(frame, corners)
            ###### DRAW ID #####
            cv2.putText(frame, "Id: " + str(ids), (0, 40), font, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            EulerAngles = rotationVectorToEulerAngles(rvec)
            EulerAngles = [round(i, 2) for i in EulerAngles]
            # cv2.putText(frame, "Attitude_angle:" + str(EulerAngles), (0, 120), font, 0.6, (0, 255, 0), 2,
            #             cv2.LINE_AA)
            tvec = tvec 
            for i in range(3):
                tvec[0][0][i] = round(tvec[0][0][i], 3)
            tvec = np.squeeze(tvec)
            # cv2.putText(frame, "Position_coordinates:" + str(tvec) + str('m'), (0, 80), font, 0.6, (0, 255, 0), 2,
            #             cv2.LINE_AA)

            Pt = [0, 0]
            for j in range(i, len(ang)):
                
                i+=1
                angles1 = ang[j]
                coords1 = cood[j]
                print('angles_data:', angles1)
                print('coords_data:', coords1)
                q1 = angles1[0]
                Pt = [coords1[0], coords1[1]]
                Pc = [tvec[0], tvec[1], tvec[2]]
                # Pc = [coords1[0], coords1[1], coords1[2]]
                Pm = [0, 0]
                offset = [0.014, 0.016, 0.222]
                imishiro = 86.06
                px = Pc[0] - offset[0]
                py = Pc[1] - offset[1]
                c1 = math.cos(q1)
                s1 = math.sin(q1)
                x = c1*px + py*s1
                y = px*s1 - c1*py

                Pm[0] = round(Pt[0] + imishiro*x, 3)
                Pm[1] = round(Pt[1] + imishiro*y, 3)
                time.sleep(0.2)
            cv2.putText(frame, "COORDS:" + str(Pt) + str('mm'), (0, 80), font, 0.6, (0, 255, 0), 2,
                        cv2.LINE_AA)
            cv2.putText(frame, "Pm:" + str(Pm) + str('mm'), (0, 200), font, 0.6, (0, 255, 0), 2,
                        cv2.LINE_AA)
            

        else:
            cv2.putText(frame, "No Ids", (0, 40), font, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        # cv2.namedWindow('frame', 0)
        cv2.resizeWindow("frame", 640, 480)

        cv2.imshow("frame", frame)
    
        key = cv2.waitKey(1)

        if key == 27:         # 按esc键退出
            print('esc break...')
            cap.release()
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    try:
        t = threading.Thread(target=target)
        t.setDaemon(True)
        t.start()
        move()
        rotationVectorToEulerAngles()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()