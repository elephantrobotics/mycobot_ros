#!/usr/bin/python
# -*- coding: UTF-8 -*-
import numpy as np
import time
import cv2, math
import tf
from tf.broadcaster import TransformBroadcaster
import rospy

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

br = TransformBroadcaster()
mtx = np.array([[629.61554535, 0, 333.57279485], [0, 631.61712266, 229.33660831], [ 0, 0, 1]])
dist = np.array(([[0.03109901, -0.0100412, -0.00944869, 0.00123176, 0.31024847]]))
cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
#num = 0
rospy.init_node("detect_markers")

while True:
    ret, frame = cap.read()
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
        cv2.putText(frame, "Attitude_angle:" + str(EulerAngles), (0, 120), font, 0.6, (0, 255, 0), 2,
                    cv2.LINE_AA)
        tvec = tvec * 1000
        for i in range(3):
            tvec[0][0][i] = round(tvec[0][0][i], 1)
        tvec = np.squeeze(tvec)
        cv2.putText(frame, "Position_coordinates:" + str(tvec) + str('mm'), (0, 80), font, 0.6, (0, 255, 0), 2,
                    cv2.LINE_AA)

        xyz = tvec / 1000
        tf_change = tf.transformations.quaternion_from_euler(0, 0, 0)
        br.sendTransform(
                    xyz, tf_change, rospy.Time.now(), "basic_shapes", "link6"
                )

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
        rotationVectorToEulerAngles()
        rospy.spin()

    except KeyboardInterrupt:
        cv2.destroyAllWindows()