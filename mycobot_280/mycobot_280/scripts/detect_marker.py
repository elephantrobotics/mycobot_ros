#!/usr/bin/env python
# encoding=utf-8
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import tf
from tf.broadcaster import TransformBroadcaster
import tf_conversions
from mycobot_communication.srv import (
    GetCoords,
    SetCoords,
    GetAngles,
    SetAngles,
    GripperStatus,
)
import math


# class Service_Coords():
#     def __init__(self):
#         self.connect_str()

#     def connect_str(self):
#         rospy.wait_for_service("get_joint_coords")
    
#         try:
#             self.get_coords = rospy.ServiceProxy("get_joint_coords", GetCoords)
#         except:
#             print("start error ...")
#             exit(1)

class ImageConverter:
    def __init__(self):
        self.br = TransformBroadcaster()
        self.bridge = CvBridge()
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        self.aruo_params = cv.aruco.DetectorParameters_create()
        # camera calibrationParams 相机校准参数
        calibrationParams = cv.FileStorage(
            "calibrationFileName.xml", cv.FILE_STORAGE_READ
        )
        # vector of distortion coefficients 失真系数
        self.dist_coeffs = calibrationParams.getNode("distCoeffs").mat()
        self.camera_matrix = None
        # subscriber, listen wether has img come in. 订阅者，监听是否有img
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.callback)
        # rospy.Rate(30)
   

    def callback(self, data):
        """Callback function.

        Process image with OpenCV, detect Mark to get the pose. Then acccording the
        pose to transforming.
        """
        # global mt
        try:
            # trans `rgb` to `gbr` for opencv. 将 `rgb` 转换为 opencv 的 `gbr`。
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        size = cv_image.shape
        focal_length = size[1]
        center = [size[1] / 2, size[0] / 2]
        print('center--->', center)
        if self.camera_matrix is None:
            # calc the camera matrix, if don't have.如果没有，则计算相机矩阵
            self.camera_matrix = np.array(
                [
                    [focal_length, 0, center[0]],
                    [0, focal_length, center[1]],
                    [0, 0, 1],
                ],
                dtype=np.float32,
            )
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        # detect aruco marker.检测 aruco 标记
        ret = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruo_params)
        corners, ids = ret[0], ret[1]
        font = cv.FONT_HERSHEY_SIMPLEX
        # process marker data.处理标记数据
        if len(corners) > 0:
            if ids is not None:
                # print('corners:', corners, 'ids:', ids)

                # detect marker pose. 检测marker位姿。
                # argument:
                #   marker corners,marker 标记
                #   marker size (meter),标记尺寸（米）
                ret = cv.aruco.estimatePoseSingleMarkers(
                    corners, 0.035, self.camera_matrix, self.dist_coeffs
                )
                # rvec:corresponding to the rotation vector of marker 旋转向量
                # tvet:corresponding to the translation vector marker 平移向量
                (rvec, tvec) = (ret[0], ret[1])
                # get rid of that nasty numpy value array error
                (rvec - tvec).any()

                print("rvec:", rvec, "tvec:", tvec)
                cv.putText(cv_image, "Id: " + str(ids[0][0]), (0, 40), font, 0.6, (0, 255, 0), 2, cv.LINE_AA)

                # just select first one detected marker.只需选择第一个检测到的标记。
                for i in range(rvec.shape[0]):
                    cv.aruco.drawDetectedMarkers(cv_image, corners)
                    # draw the axes  绘制坐标轴
                    cv.aruco.drawAxis(
                        cv_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec[i, :, :],
                        tvec[i, :, :],
                        0.03,
                    )

                xyz = tvec[0, 0, :]
                # xyz = [xyz[0] , xyz[1], xyz[2]]
                xyz = [xyz[0] - 0.045, xyz[1], xyz[2] - 0.03]

                tsvec = tvec
                for i in range(3):
                    tsvec[0][0][i] = round(tvec[0][0][i], 1)
                tsvec = np.squeeze(tsvec)
                
                # cv.putText(cv_image, "position_coords:" + str(tsvec) + str('m'), (0, 80), font, 0.6, (0, 255, 0), 2)
               
                # get euler angles. 获取欧拉角
                euler = rvec[0, 0, :]
                # Euler angle to quaternion in ros. ros中欧拉角转四元数
                tf_change = tf.transformations.quaternion_from_euler(
                    #  euler[0], euler[1], euler[2]
                    0, 0, 0
                )
                print("tf_change:", tf_change)
                print('xyz:',xyz)
                #xyz = [0.1, 0.1, 0.25]
                # trans pose according [joint1]，根据 [joint1] 变换姿势 rospy.Time.now() Broadcast tf transformation
                self.br.sendTransform(
                    xyz, tf_change, rospy.Time.now(), "basic_shapes", "link6"
                )
        else:
            cv.putText(cv_image, "No Ids", (0, 40), font, 0.6, (0, 255, 0), 2, cv.LINE_AA)    
                
        # [x, y, z, -172, 3, -46.8]
        cv.imshow("Image", cv_image)

        cv.waitKey(3)
        try:
            pass
        except CvBridgeError as e:
            print(e)
        # rospy.Rate(30).sleep()

if __name__ == "__main__":
    try:
        # global mt
        
        rospy.init_node("detect_marker")
        rospy.loginfo("Starting cv_bridge_test node")
        # mt = Service_Coords()
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv.destroyAllWindows()
