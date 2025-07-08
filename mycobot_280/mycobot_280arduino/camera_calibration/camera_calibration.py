import cv2
from uvc_camera import UVCCamera
import stag
import numpy as np
import json
import time
import os
from scipy.linalg import svd
from pymycobot import *
from marker_utils import *
import shutil
import glob


ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyAMA*')
print(ports)
if ports:
    arm_port = ports[0]
else:
    raise Exception("No MyCobot device found")


mc = MyCobot280(port=arm_port, baudrate=1000000)  # 设置端口
            
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

        self.origin_mycbot_horizontal = [-90, -35.85, -52.91, 88.59, 0, 0.0]
        self.origin_mycbot_level = [-90, 5, -104, 14, 0, 0]
        
        self.IDENTIFY_LEN = 300
   
        # Initialize EyesInHand_matrix to None or load from a document if available
        self.EyesInHand_matrix = None
        file_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.matrix_file_path = os.path.join(file_dir, "config","EyesInHand_matrix.json")
    
        self.load_matrix()

    def save_matrix(self, filename="EyesInHand_matrix.json"):
        # Save the EyesInHand_matrix to a JSON file
        if self.EyesInHand_matrix is not None:
            with open(filename, 'w') as f:
                json.dump(self.EyesInHand_matrix.tolist(), f)
                
            try:
                # 复制文件到目标路径
                shutil.copy(filename, self.matrix_file_path)
                print(f"File copied to {self.matrix_file_path}")
            except IOError as e:
                print(f"Failed to copy file: {e}")
    
    def load_matrix(self, filename="EyesInHand_matrix.json"):
        # Load the EyesInHand_matrix from a JSON file, if it exists
        try:
            with open(filename, 'r') as f:
                self.EyesInHand_matrix = np.array(json.load(f))
        except FileNotFoundError:
            print("Matrix file not found. EyesInHand_matrix will be initialized later.")
            
    def CvtRotationMatrixToEulerAngle(self, pdtRotationMatrix):
        pdtEulerAngle = np.zeros(3)
        pdtEulerAngle[2] = np.arctan2(pdtRotationMatrix[1, 0], pdtRotationMatrix[0, 0])
        fCosRoll = np.cos(pdtEulerAngle[2])
        fSinRoll = np.sin(pdtEulerAngle[2])
        pdtEulerAngle[1] = np.arctan2(-pdtRotationMatrix[2, 0],
                                      (fCosRoll * pdtRotationMatrix[0, 0]) + (fSinRoll * pdtRotationMatrix[1, 0]))
        pdtEulerAngle[0] = np.arctan2((fSinRoll * pdtRotationMatrix[0, 2]) - (fCosRoll * pdtRotationMatrix[1, 2]),
                                      (-fSinRoll * pdtRotationMatrix[0, 1]) + (fCosRoll * pdtRotationMatrix[1, 1]))
        return pdtEulerAngle

    # 将欧拉角转为旋转矩阵
    def CvtEulerAngleToRotationMatrix(self, ptrEulerAngle):
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

    
    def eyes_in_hand_calculate(self, pose, tbe, Mc, Mr):

        pose,Mr =  map(np.array, [pose,Mr])
        # 将角度从度数转换为弧度
        euler = pose * np.pi / 180
        Rbe = self.CvtEulerAngleToRotationMatrix(euler)
        Reb = Rbe.T

        A = np.empty((3, 0))
        b_comb = np.empty((3, 0))
        
        r = tbe.shape[0]
        
        for i in range(1, r):
            A = np.hstack((A, (Mc[i, :].reshape(3, 1) - Mc[0, :].reshape(3, 1))))
            b_comb = np.hstack((b_comb, (tbe[0, :].reshape(3, 1) - tbe[i, :].reshape(3, 1))))
        
        b = Reb @ b_comb
        U, _, Vt = svd(A @ b.T)
        Rce = Vt.T @ U.T
        
        tbe_sum = np.sum(tbe, axis=0)
        Mc_sum = np.sum(Mc, axis=0)
        
        tce = Reb @ (Mr.reshape(3, 1) - (1/r) * tbe_sum.reshape(3, 1) - (1/r) * (Rbe @ Rce @ Mc_sum.reshape(3, 1)))
        tce[2] -= self.IDENTIFY_LEN #用于保持识别距离

        EyesInHand_matrix = np.vstack((np.hstack((Rce, tce)), np.array([0, 0, 0, 1])))
        print("EyesInHand_matrix = ", EyesInHand_matrix)
        return EyesInHand_matrix

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
             marker_pos_pack, ids = self.stag_identify()
        # print("Camera coords = ", marker_pos_pack)
        # cv2.imshow("rrrr", frame)
        # cv2.waitKey(1)
        return marker_pos_pack, ids
    
    def Matrix_identify(self, ml):
        ml.send_angles(self.origin_mycbot_level, 50)  # 移动到观测点
        self.wait()
        input("make sure camera can observe the stag, enter any key quit")
        coords = ml.get_coords()
        pose = coords[3:6]
        print(pose)
        # self.camera_open_loop()
        Mc1,tbe1,pos1 = self.reg_get(ml)
        ml.send_coord(1, coords[0] + 50, 30)
        self.wait()
        Mc2,tbe2,pos2 = self.reg_get(ml)
        ml.send_coord(3, coords[2] + 20, 30)
        self.wait()
        Mc3,tbe3,pos3 = self.reg_get(ml)
        ml.send_coord(2, coords[1] + 20, 30)
        self.wait()
        Mc4,tbe4,pos4 = self.reg_get(ml)
        ml.send_coord(1, coords[0] + 20, 30)
        self.wait()
        Mc5,tbe5,pos5 = self.reg_get(ml)
        tbe = np.vstack([tbe1, tbe2, tbe3, tbe4, tbe5])
        Mc = np.vstack([Mc1, Mc2, Mc3, Mc4, Mc5])
        state = None
        if self.EyesInHand_matrix is not None:
            state = True
            pos = np.vstack([pos1, pos2, pos3, pos4, pos5])
            r = pos.shape[0]
            for i in range(1, r):
                for j in range(3):
                    err = abs(pos[i][j] - pos[0][j])
                    if(err > 10):
                        state = False
                        # print("matrix error")
        return pose, tbe, Mc, state

    def Eyes_in_hand_calibration(self, ml):
        ml.set_end_type(0)
        pose, tbe, Mc, state = self.Matrix_identify(ml)
        if(state == True):
            print("Calibration Complete EyesInHand_matrix = ", self.EyesInHand_matrix)
            return
        
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

        self.EyesInHand_matrix = self.eyes_in_hand_calculate(pose, tbe, Mc, Mr)
        print("EyesInHand_matrix = ", self.EyesInHand_matrix)
        self.save_matrix()  # Save the matrix to a file after calculating it
        print("save successe, wait to verify")
        
        pose, tbe, Mc, state = self.Matrix_identify(ml)
        if state != True:
            self.EyesInHand_matrix = self.eyes_in_hand_calculate(pose, tbe, Mc, Mr)

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
    
    def Eyes_in_hand(self, coord, camera, Matrix_TC):
        Position_Camera = np.transpose(camera[:3])  # 相机坐标
        Matrix_BT = self.Transformation_matrix(coord)  # 机械臂坐标矩阵

        Position_Camera = np.append(Position_Camera, 1)  # 物体坐标（相机系）
        Position_B = Matrix_BT @ Matrix_TC @ Position_Camera  # 物体坐标（基坐标系）
        return Position_B
    
    def stag_robot_identify(self, ml):
        marker_pos_pack,ids = self.stag_identify()
        target_coords = ml.get_coords() # 获取机械臂当前坐标
        while (target_coords is None):
            target_coords = ml.get_coords()
        # print("current_coords", target_coords)
        cur_coords = np.array(target_coords.copy())
        cur_coords[-3:] *= (np.pi / 180)  # 将角度值转为弧度值
        fact_bcl = self.Eyes_in_hand(cur_coords, marker_pos_pack, self.EyesInHand_matrix)  # 通过矩阵变化将物体坐标(相机系)转成(基坐标系)
        
        for i in range(3):
            target_coords[i] = fact_bcl[i]
        
        return target_coords,ids
    
    def reg_get(self, ml):
        target_coords = None
        for i in range(30):
            Mc_all,_ = self.stag_identify()
        if self.EyesInHand_matrix is not None:
            target_coords,_ = self.stag_robot_identify(ml)

        tbe_all = ml.get_coords() # 获取机械臂当前坐标
        while (tbe_all is None):
            tbe_all = ml.get_coords()

        tbe = np.array(tbe_all[0:3])
        Mc = np.array(Mc_all[0:3])
        print("tbe = ", tbe)
        print("Mc = ", Mc)
        return Mc,tbe,target_coords


if __name__ == "__main__":
    if mc.is_power_on()==0:
        mc.power_on()
    camera_params = np.load("camera_params.npz")  # 相机配置文件
    mtx, dist = camera_params["mtx"], camera_params["dist"]
    m = camera_detect(0, 32, mtx, dist)
    # tool_len = 20
    # mc.set_tool_reference([0, 0, tool_len, 0, 0, 0])
    # mc.set_end_type(0)

    m.Eyes_in_hand_calibration(mc)  #手眼标定
    
