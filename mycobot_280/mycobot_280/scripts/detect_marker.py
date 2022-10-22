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
        calibrationParams = cv.FileStorage(
            "calibrationFileName.xml", cv.FILE_STORAGE_READ
        )
        self.dist_coeffs = calibrationParams.getNode("distCoeffs").mat()
        self.camera_matrix = None
        # subscriber, listen wether has img come in. 订阅者，监听是否有img
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.callback)

   

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
        # process marker data.处理标记数据
        if len(corners) > 0:
            if ids is not None:
                # print('corners:', corners, 'ids:', ids)

                # detect marker pose. 检测marker位姿。
                # argument:
                #   marker corners,标记角
                #   marker size (meter),标记尺寸（米）
                ret = cv.aruco.estimatePoseSingleMarkers(
                    corners, 0.05, self.camera_matrix, self.dist_coeffs
                )
                (rvec, tvec) = (ret[0], ret[1])
                (rvec - tvec).any()

                print("rvec:", rvec, "tvec:", tvec)

                # just select first one detected marker.只需选择第一个检测到的标记。
                for i in range(rvec.shape[0]):
                    cv.aruco.drawDetectedMarkers(cv_image, corners)
                    cv.aruco.drawAxis(
                        cv_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec[i, :, :],
                        tvec[i, :, :],
                        0.03,
                    )

                xyz = tvec[0, 0, :]
                xyz = [xyz[0] - 0.045, xyz[1], xyz[2] - 0.03]

                tsvec = tvec
                for i in range(3):
                    tsvec[0][0][i] = round(tvec[0][0][i], 1)
                tsvec = np.squeeze(tsvec)
                # font = cv.FONT_HERSHEY_SIMPLEX
                # cv.putText(cv_image, "position_coords:" + str(tsvec) + str('m'), (0, 80), font, 0.6, (0, 255, 0), 2)
                # try:
                    
                #     c = mt.get_coords()
                #     mc_coords = [round(c.x, 2), round(c.y, 2), round(c.z, 2), round(c.rx, 2), round(c.ry, 2), round(c.rz), 2]
                # except Exception as e:
                #     print(e)
                # cv.putText(cv_image, "mycobot:" + str(mc_coords[:3]) + str('mm'), (0, 200), font, 0.6, (0, 255, 0), 2,
                #     cv.LINE_AA)

                # Pt = mc_coords[:3]
                # Pc = [tsvec[0], tsvec[1], tsvec[2]]
                # Pm = [0, 0]
            
                # offset = [-0.045, -0.2228, 0]
                # imishiro = 58.43
                # Pm[0] = round(Pt[0] + imishiro * (Pc[1] - offset[0]), 3)
                # Pm[1] = round(Pt[1] + imishiro * (Pc[0] - offset[1]), 3)
                # cv.putText(cv_image, "marker:" + str(Pm) + str('mm'), (0, 120), font, 0.6, (0, 255, 0), 2,
                #             cv.LINE_AA)
                # get quaternion for ros. 为ros获取四元数
                euler = rvec[0, 0, :]
                tf_change = tf.transformations.quaternion_from_euler(
                    euler[0], euler[1], euler[2]
                )
                print("tf_change:", tf_change)
                print('xyz:',xyz)

                # trans pose according [joint1]，根据 [joint1] 变换姿势
                self.br.sendTransform(
                    xyz, tf_change, rospy.Time.now(), "basic_shapes", "link6"
                )
                
        # [x, y, z, -172, 3, -46.8]
        cv.imshow("Image", cv_image)

        cv.waitKey(3)
        try:
            pass
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    try:
        # global mt
        # mt = Service_Coords()
        rospy.init_node("detect_marker")
        rospy.loginfo("Starting cv_bridge_test node")
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv.destroyAllWindows()
