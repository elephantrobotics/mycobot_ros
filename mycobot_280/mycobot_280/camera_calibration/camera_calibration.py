import cv2
from uvc_camera import UVCCamera
import stag
import numpy as np
import json
import time
from scipy.linalg import svd
from pymycobot import *


mc = MyCobot("/dev/ttyUSB0")  # 设置端口
            
np.set_printoptions(suppress=True, formatter={'float_kind': '{:.2f}'.format})



class camera_detect:
    #Camera parameter initialize
    def __init__(self, camera_id, marker_size, mtx, dist):
        self.camera_id = camera_id
        self.mtx = mtx
        self.dist = dist
        self.marker_size = marker_size
        self.camera = UVCCamera(self.camera_id, self.mtx, self.dist)
        self.camera_open()

        self.origin_mycbot_horizontal = [0,60,-60,0,0,0]
        self.origin_mycbot_level = [0, 5, -104, 14, 0, 90]
   
        # Initialize EyesInHand_matrix to None or load from a document if available
        self.EyesInHand_matrix = None
        self.load_matrix()

    def save_matrix(self, filename="EyesInHand_matrix.json"):
        # Save the EyesInHand_matrix to a JSON file
        if self.EyesInHand_matrix is not None:
            with open(filename, 'w') as f:
                json.dump(self.EyesInHand_matrix.tolist(), f)
    
    def load_matrix(self, filename="EyesInHand_matrix.json"):
        # Load the EyesInHand_matrix from a JSON file, if it exists
        try:
            with open(filename, 'r') as f:
                self.EyesInHand_matrix = np.array(json.load(f))
        except FileNotFoundError:
            print("Matrix file not found. EyesInHand_matrix will be initialized later.")

    def wait(self):
        time.sleep(0.5)
        while(mc.is_moving() == 1):
            time.sleep(0.2)
    
    def camera_open(self):
        self.camera.capture()  # 打开摄像头

    # 获取物体坐标(相机系)
    def calc_markers_base_position(self, corners, ids):
        if len(corners) == 0:
            return []
        rvecs, tvecs = solve_marker_pnp(corners, self.marker_size, self.mtx, self.dist)  # 通过二维码角点获取物体旋转向量和平移向量
        for i, tvec, rvec in zip(ids, tvecs, rvecs):
            tvec = tvec.squeeze().tolist()
            rvec = rvec.squeeze().tolist()
            rotvector = np.array([[rvec[0], rvec[1], rvec[2]]])
            Rotation = cv2.Rodrigues(rotvector)[0]  # 将旋转向量转为旋转矩阵
            Euler = self.CvtRotationMatrixToEulerAngle(Rotation)  # 将旋转矩阵转为欧拉角
            target_coords = np.array([tvec[0], tvec[1], tvec[2], Euler[0], Euler[1], Euler[2]])  # 物体坐标(相机系)
        return target_coords

    
    def eyes_in_hand_calculate(self, pose, tbe1, Mc1, tbe2, Mc2, tbe3, Mc3, Mr):

        tbe1, Mc1, tbe2, Mc2, tbe3, Mc3, Mr = map(np.array, [tbe1, Mc1, tbe2, Mc2, tbe3, Mc3, Mr])
        # Convert pose from degrees to radians
        euler = np.array(pose) * np.pi / 180
        Rbe = self.CvtEulerAngleToRotationMatrix(euler)
        print("Rbe", Rbe)
        Reb = Rbe.T
        
        A = np.hstack([(Mc2 - Mc1).reshape(-1, 1), 
                    (Mc3 - Mc1).reshape(-1, 1), 
                    (Mc3 - Mc2).reshape(-1, 1)])
        
        b = Reb @ np.hstack([(tbe1 - tbe2).reshape(-1, 1), 
                            (tbe1 - tbe3).reshape(-1, 1), 
                            (tbe2 - tbe3).reshape(-1, 1)])
        
        print("A = ", A)
        print("B = ", b)
        U, S, Vt = svd(A @ b.T)
        Rce = Vt.T @ U.T
        
        tce = Reb @ (Mr - (1/3)*(tbe1 + tbe2 + tbe3) - (1/3)*(Rbe @ Rce @ (Mc1 + Mc2 + Mc3)))
        
        eyes_in_hand_matrix = np.vstack([np.hstack([Rce, tce.reshape(-1, 1)]), np.array([0, 0, 0, 1])])
        
        return eyes_in_hand_matrix

    # 读取Camera坐标（单次）
    def stag_identify(self):
        self.camera.update_frame()  # 刷新相机界面
        frame = self.camera.color_frame()  # 获取当前帧
        (corners, ids, rejected_corners) = stag.detectMarkers(frame, 11)  # 获取画面中二维码的角度和id
        # 绘制检测到的标记及其ID
        stag.drawDetectedMarkers(frame, corners, ids)
        # 绘制被拒绝的候选区域，颜色设为红色
        stag.drawDetectedMarkers(frame, rejected_corners, border_color=(255, 0, 0))
        marker_pos_pack = self.calc_markers_base_position(corners, ids)  # 获取物的坐标(相机系)
        if(len(marker_pos_pack) == 0):
            marker_pos_pack = self.stag_identify()
        # print("Camera coords = ", marker_pos_pack)
        # cv2.imshow("rrrr", frame)
        # cv2.waitKey(1)
        return marker_pos_pack

    def Eyes_in_hand_calibration(self, ml):
        ml.send_angles(self.origin_mycbot_level, 50)  # 移动到观测点
        self.wait()
        input("make sure camera can observe the stag, enter any key quit")
        coords = ml.get_coords()
        pose = coords[3:6]
        print(pose)
        # self.camera_open_loop()
        Mc1,tbe1 = self.reg_get(ml)
        ml.send_coord(1, coords[0] + 30, 30)
        self.wait()
        Mc2,tbe2 = self.reg_get(ml)
        ml.send_coord(1, coords[0] - 10, 30)
        self.wait()
        ml.send_coord(3, coords[2] + 20, 30)
        self.wait()
        Mc3,tbe3 = self.reg_get(ml)

        input("Move the end of the robot arm to the calibration point, press any key to release servo")
        ml.release_all_servos()
        input("focus servo and get current coords")
        ml.power_on()
        time.sleep(1)
        coords = ml.get_coords()
        while len(coords) == 0:
            coords = ml.get_coords()
        Mr = coords[0:3]
        print(Mr)

        self.EyesInHand_matrix = self.eyes_in_hand_calculate(pose, tbe1, Mc1, tbe2, Mc2, tbe3, Mc3, Mr)
        print("EyesInHand_matrix = ", self.EyesInHand_matrix)
        self.save_matrix()  # Save the matrix to a file after calculating it
        print("save successe")
    
    def reg_get(self, ml):
        for i in range(50):
            Mc_all = self.stag_identify()
        tbe_all = ml.get_coords() # 获取机械臂当前坐标
        while (tbe_all is None):
            tbe_all = ml.get_coords()

        tbe = tbe_all[0:3]
        Mc = Mc_all[0:3]
        print("tbe = ", tbe)
        print("Mc = ", Mc)
        return Mc,tbe


if __name__ == "__main__":
    if mc.is_power_on()==0:
        mc.power_on()
    camera_params = np.load("camera_params.npz")  # 相机配置文件
    mtx, dist = camera_params["mtx"], camera_params["dist"]
    m = camera_detect(0, 32, mtx, dist)
    tool_len = 20
    mc.set_tool_reference([0, 0, tool_len, 0, 0, 0])
    mc.set_end_type(1)

    m.Eyes_in_hand_calibration(mc)  #手眼标定
    
