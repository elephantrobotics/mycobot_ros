#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from numpy.typing import NDArray, ArrayLike
import stag
import os
import json
import time
import threading
from mycobot_communication.msg import MycobotAngles, MycobotSetAngles, MycobotCoords, MycobotSetCoords, MycobotSetEndType, MycobotSetFreshMode, MycobotSetToolReference, MycobotSetVisionMode
from visualization_msgs.msg import Marker

np.set_printoptions(suppress=True, formatter={'float_kind': '{:.2f}'.format})

class STAGRecognizer:
    def __init__(self):
        rospy.init_node('stag_recognizer', anonymous=True)
        self.bridge = CvBridge()
        self.tool_len = 20

        self.marker_size = 32
        self.origin_mycbot_horizontal = [0,60,-60,0,0,-40]

        self.EyesInHand_matrix = None
        # 订阅摄像头话题
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        
        # 获取config文件目录并设置相机参数文件路径
        file_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        params_file_path = os.path.join(file_dir, "config","camera_params.npz")
        print(params_file_path)
        matrix_file_path = os.path.join(file_dir, "config","EyesInHand_matrix.json")
        self.load_matrix(filename=matrix_file_path)
        print(matrix_file_path)
        # 加载相机参数
        try:
            camera_params = np.load(params_file_path)
            self.mtx, self.dist = camera_params["mtx"], camera_params["dist"]
        except FileNotFoundError:
            rospy.logerr(f"Camera parameters file not found: {params_file_path}")
            raise
        
        self.current_frame = None
        
        # 创建发布者，发布机械臂坐标和角度
        self.coords_pub = rospy.Publisher('mycobot/coords_goal', MycobotSetCoords, queue_size=5)
        self.angles_pub = rospy.Publisher('mycobot/angles_goal', MycobotSetAngles, queue_size=5)
        self.fresh_mode_pub = rospy.Publisher('mycobot/fresh_mode_status', MycobotSetFreshMode, queue_size=5)
        self.end_type_pub = rospy.Publisher('mycobot/end_type_status', MycobotSetEndType, queue_size=5)
        self.tool_reference_pub = rospy.Publisher('mycobot/tool_reference_goal', MycobotSetToolReference, queue_size=5)
        self.vision_mode_pub = rospy.Publisher('mycobot/vision_mode_status', MycobotSetVisionMode, queue_size=5)
        
        # 创建订阅者，订阅机械臂的真实坐标和角度
        rospy.Subscriber('mycobot/coords_real', MycobotCoords, self.coords_callback)
        
        self.current_coords = None
        self.current_angles = None
        self.lock = threading.Lock()
        self.set_tool_reference([0, 0, self.tool_len, 0, 0, 0])
        self.set_end_type(1)
        
        # init a node and a publisher
        # rospy.init_node("marker", anonymous=True)
        self.pub = rospy.Publisher('cube', Marker, queue_size=1)
        # init a Marker
        self.marker = Marker()
        self.marker.header.frame_id = "joint1"
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
        
    def load_matrix(self, filename="EyesInHand_matrix.json"):
        # Load the EyesInHand_matrix from a JSON file, if it exists
        try:
            with open(filename, 'r') as f:
                self.EyesInHand_matrix = np.array(json.load(f))
        except FileNotFoundError:
            print("Matrix file not found. EyesInHand_matrix will be initialized later.")

    # publish marker
    def pub_marker(self, x, y, z=0.03):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.color.g = 0
        self.pub.publish(self.marker)
    
    def coords_callback(self, data):
        # 获取机械臂当前的坐标，并保留小数点后两位
        with self.lock:
            self.current_coords = [round(data.x, 2), round(data.y, 2), round(data.z, 2),
                                round(data.rx, 2), round(data.ry, 2), round(data.rz, 2)]
            self.coords_updated = True
            # rospy.loginfo(f"Current coords11111: {self.current_coords}")
        
    def send_angles(self, angles, speed):
        msg = MycobotSetAngles()
        msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6 = angles
        msg.speed = speed
        self.angles_pub.publish(msg)
        
    def send_coords(self, coords, speed, model):
        # 创建 MycobotSetCoords 消息对象
        msg = MycobotSetCoords()
        
        # coords 是一个包含 [x, y, z, rx, ry, rz] 的列表
        if len(coords) != 6:
            raise ValueError("coords must be a list of 6 elements")
        msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = coords
        msg.speed = speed
        msg.model = model

        # 发布消息
        self.coords_pub.publish(msg)

        
    def get_coords(self):
        with self.lock:
            if self.coords_updated:
                self.coords_updated = False
                return self.current_coords.copy()
            return []
        
    def set_fresh_mode(self, mode):
        msg = MycobotSetFreshMode()
        msg.Status = mode
        self.fresh_mode_pub.publish(msg)
        
    def set_vision_mode(self, mode):
        msg = MycobotSetVisionMode()
        msg.Status = mode
        self.vision_mode_pub.publish(msg)
        
    def set_end_type(self, end_type):
        msg = MycobotSetEndType()
        msg.Status = end_type
        self.end_type_pub.publish(msg)

    def set_tool_reference(self, coords):
        msg = MycobotSetToolReference()
        msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = coords
        self.tool_reference_pub.publish(msg)


    def solve_marker_pnp(self, corners: NDArray, marker_size: int, mtx: NDArray, dist: NDArray):
        """
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        """
        marker_points = np.array(
            [
                [-marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, -marker_size / 2, 0],
                [-marker_size / 2, -marker_size / 2, 0],
            ],
            dtype=np.float32,
        )
        rvecs = []
        tvecs = []
        for corner in corners:
            retval, rvec, tvec = cv2.solvePnP(
                marker_points,
                corner,
                mtx,
                dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if retval:
                rvecs.append(rvec)
                tvecs.append(tvec)

        rvecs = np.array(rvecs)  # type: ignore
        tvecs = np.array(tvecs)  # type: ignore
        (rvecs - tvecs).any()  # type: ignore
        return rvecs, tvecs

    def image_callback(self, data):
        try:
            # 将 ROS 图像消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_frame = cv_image
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        # 应用相机校正
        # frame_undistorted = cv2.undistort(cv_image, self.mtx, self.dist, None, self.mtx)

        # 检测 STAG 标记
        corners, ids, rejected_corners = stag.detectMarkers(cv_image, 11)
        # 绘制检测到的标记及其ID
        stag.drawDetectedMarkers(cv_image, corners, ids)
        # 绘制被拒绝的候选区域，颜色设为红色
        stag.drawDetectedMarkers(cv_image, rejected_corners, border_color=(255, 0, 0))
        
        # cv2.imshow("STAG Detection", cv_image)
        # cv2.waitKey(1)
        
    def calc_markers_base_position(self, corners, ids):
        """获取物体坐标(相机系)

        Args:
            corners (_type_): _description_
            ids (_type_): _description_

        Returns:
            _type_: _description_
        """
        if len(corners) == 0:
            return []
        # 通过二维码角点获取物体旋转向量和平移向量
        rvecs, tvecs = self.solve_marker_pnp(corners, self.marker_size, self.mtx, self.dist)
        for i, tvec, rvec in zip(ids, tvecs, rvecs):
            tvec = tvec.squeeze().tolist()
            rvec = rvec.squeeze().tolist()
            rotvector = np.array([[rvec[0], rvec[1], rvec[2]]])
            # 将旋转向量转为旋转矩阵
            Rotation = cv2.Rodrigues(rotvector)[0]  
            # 将旋转矩阵转为欧拉角
            Euler = self.CvtRotationMatrixToEulerAngle(Rotation)  
            # 物体坐标(相机系)
            target_coords = np.array([tvec[0], tvec[1], tvec[2], Euler[0], Euler[1], Euler[2]])
        return target_coords
    
    def CvtRotationMatrixToEulerAngle(self, pdtRotationMatrix):
        """将旋转矩阵转为欧拉角

        Args:
            pdtRotationMatrix (_type_): _description_

        Returns:
            _type_: _description_
        """
        pdtEulerAngle = np.zeros(3)
        pdtEulerAngle[2] = np.arctan2(pdtRotationMatrix[1, 0], pdtRotationMatrix[0, 0])
        fCosRoll = np.cos(pdtEulerAngle[2])
        fSinRoll = np.sin(pdtEulerAngle[2])
        pdtEulerAngle[1] = np.arctan2(-pdtRotationMatrix[2, 0],
                                      (fCosRoll * pdtRotationMatrix[0, 0]) + (fSinRoll * pdtRotationMatrix[1, 0]))
        pdtEulerAngle[0] = np.arctan2((fSinRoll * pdtRotationMatrix[0, 2]) - (fCosRoll * pdtRotationMatrix[1, 2]),
                                      (-fSinRoll * pdtRotationMatrix[0, 1]) + (fCosRoll * pdtRotationMatrix[1, 1]))
        return pdtEulerAngle
    
    def CvtEulerAngleToRotationMatrix(self, ptrEulerAngle):
        """将欧拉角转为旋转矩阵

        Args:
            ptrEulerAngle (_type_): _description_

        Returns:
            _type_: _description_
        """
        ptrSinAngle = np.sin(ptrEulerAngle)
        ptrCosAngle = np.cos(ptrEulerAngle)
        ptrRotationMatrix = np.zeros((3, 3))
        ptrRotationMatrix[0, 0] = ptrCosAngle[2] * ptrCosAngle[1]
        ptrRotationMatrix[0, 1] = ptrCosAngle[2] * ptrSinAngle[1] * ptrSinAngle[0] - ptrSinAngle[2] * ptrCosAngle[0]
        ptrRotationMatrix[0, 2] = ptrCosAngle[2] * ptrSinAngle[1] * ptrCosAngle[0] + ptrSinAngle[2] * ptrSinAngle[0]
        ptrRotationMatrix[1, 0] = ptrSinAngle[2] * ptrCosAngle[1]
        ptrRotationMatrix[1, 1] = ptrSinAngle[2] * ptrSinAngle[1] * ptrSinAngle[0] + ptrCosAngle[2] * ptrCosAngle[0]
        ptrRotationMatrix[1, 2] = ptrSinAngle[2] * ptrSinAngle[1] * ptrCosAngle[0] - ptrCosAngle[2] * ptrSinAngle[0]
        ptrRotationMatrix[2, 0] = -ptrSinAngle[1]
        ptrRotationMatrix[2, 1] = ptrCosAngle[1] * ptrSinAngle[0]
        ptrRotationMatrix[2, 2] = ptrCosAngle[1] * ptrCosAngle[0]
        return ptrRotationMatrix
    
    def Transformation_matrix(self,coord):
        """坐标转换为齐次变换矩阵

        Args:
            coord (_type_): (x,y,z,rx,ry,rz)

        Returns:
            _type_: _description_
        """
        position_robot = coord[:3]
        pose_robot = coord[3:]
        # 将欧拉角转为旋转矩阵
        RBT = self.CvtEulerAngleToRotationMatrix(pose_robot)  
        PBT = np.array([[position_robot[0]],
                        [position_robot[1]],
                        [position_robot[2]]])
        temp = np.concatenate((RBT, PBT), axis=1)
        array_1x4 = np.array([[0, 0, 0, 1]])
        # 将两个数组按行拼接起来
        matrix = np.concatenate((temp, array_1x4), axis=0)  
        return matrix

    def Eyes_in_hand(self, coord, marker_positions, Matrix_TC):
        # 相机坐标
        Position_Camera = np.transpose(marker_positions[:3])
        # 机械臂坐标矩阵
        Matrix_BT = self.Transformation_matrix(coord)
        # 物体坐标（相机系）
        Position_Camera = np.append(Position_Camera, 1)

        # 物体坐标（基坐标系）
        Position_B = Matrix_BT @ Matrix_TC @ Position_Camera
        return Position_B
    
    def waitl(self, ml):
        """等待机械臂运行结束

        Args:
            ml (_type_): _description_
        """
        time.sleep(0.2)
        while ml.is_moving():
            time.sleep(0.03)
    
    def stag_identify(self):
        """读取Camera坐标（单次）

        Returns:
            _type_: _description_
        """
        try:
            if self.current_frame is None:
                rospy.logwarn("No image received yet")
                return []
            # 获取当前帧
            frame = self.current_frame
            # 获取画面中二维码的角度和id
            corners, ids, rejected_corners = stag.detectMarkers(frame, 11)
            # 获取物的坐标(相机系)
            marker_pos_pack = self.calc_markers_base_position(corners, ids)
            if len(marker_pos_pack) == 0 and not rospy.is_shutdown():
                # rospy.logwarn("No markers detected")
                marker_pos_pack = self.stag_identify()  # 递归调用

            # print("Camera coords = ", marker_pos_pack)
            return marker_pos_pack
        except RecursionError:
            # rospy.logerr("Recursion depth exceeded in marker detection")
            return [0, 0, 0, 0]  # 返回默认值
    
    def vision_trace(self, mode, ml):
        sp = 40
        #水平面抓取
        if mode == 0:
            # 移动到观测点
            ml.send_angles(self.origin_mycbot_horizontal, sp)
            # 等待机械臂运动结束
            self.waitl(ml)
            input("Enter any key to start trace")

            target_coords = self.stag_robot_identify(ml)
            print(target_coords)

            time.sleep(1)
            # 机械臂移动到二维码前方
            ml.send_coords(target_coords, 30)
            # 等待机械臂运动结束
            self.waitl(ml)
            
    def stag_robot_identify(self):
        marker_pos_pack = self.stag_identify()
        # 如果返回的是默认值，直接退出函数，不返回任何数据
        # if marker_pos_pack == [0, 0, 0, 0]:
        if np.array_equal(marker_pos_pack, [0, 0, 0, 0]):
            rospy.logwarn("No markers detected, skipping processing")
            return None  # 直接返回 None
        target_coords = self.get_coords()
        while len(target_coords)==0:
            target_coords = self.get_coords()
        # print("Current coords:", target_coords)
        cur_coords = np.array(target_coords.copy())
        cur_coords[-3:] *= (np.pi / 180)
        fact_bcl = self.Eyes_in_hand(cur_coords, marker_pos_pack, self.EyesInHand_matrix)
        for i in range(3):
            target_coords[i] = fact_bcl[i]
        return target_coords
    
    def coord_limit(self, coords):
        min_coord = [100, -150, 0]
        max_coord = [400, 150, 400]
        for i in range(3):
            if(coords[i] < min_coord[i]):
                coords[i] = min_coord[i]

            if(coords[i] > max_coord[i]):
                coords[i] = max_coord[i]
    
    def vision_trace_loop(self):
        self.set_fresh_mode(1)
        time.sleep(1)
        # self.set_vision_mode(0)
        # 移动到观测点
        self.send_angles(self.origin_mycbot_horizontal, 50)  
        time.sleep(3)
        origin = self.get_coords()

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            target_coords = self.stag_robot_identify()
             # 如果没有返回目标坐标，跳过本次循环
            if target_coords is None:
                continue  # 跳过这次循环，等下次识别
            target_coords[0] -= 300
            rospy.loginfo('Target Coords: %s', target_coords)
            self.coord_limit(target_coords)
            for i in range(3):
                target_coords[i+3] = origin[i+3]
            self.pub_marker(target_coords[0]/1000.0, target_coords[1]/1000.0, target_coords[2]/1000.0)
            self.send_coords(target_coords, 30, 0)  # 机械臂移动到二维码前方
            rate.sleep()
    
    
if __name__ == '__main__': 
    try:
        sr = STAGRecognizer()
        sr.vision_trace_loop()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
        sr.set_vision_mode(2)
        cv2.destroyAllWindows()
