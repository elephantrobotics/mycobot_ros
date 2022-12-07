#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Tkinter as tk
from mycobot_communication.srv import GetCoords, SetCoords, GetAngles, SetAngles, GripperStatus
import rospy
import time
from rospy import ServiceException


class Window:
    def __init__(self, handle):
        self.win = handle
        self.win.resizable(0, 0)  # fixed window size，固定窗口大小

        self.model = 0
        self.speed = rospy.get_param("~speed", 50)

        # set default speed，设置默认速度
        self.speed_d = tk.StringVar()
        self.speed_d.set(str(self.speed))
        # print(self.speed)
        self.connect_ser()

        # Get the data of the robotic arm，获取机械臂数据
        self.record_coords = [0, 0, 0, 0, 0, 0, self.speed, self.model]
        self.res_angles = [0, 0, 0, 0, 0, 0, self.speed, self.model]
        self.get_date()

        # get screen width and height.获取屏幕宽度和高度
        self.ws = self.win.winfo_screenwidth()  # width of the screen
        self.hs = self.win.winfo_screenheight()  # height of the screen
        # calculate x and y coordinates for the Tk root window
        # 计算 Tk 根窗口的 x 和 y 坐标
        x = (self.ws / 2) - 190
        y = (self.hs / 2) - 250
        self.win.geometry("430x400+{}+{}".format(int(x), int(y)))
        # layout,布局
        self.set_layout()
        # input section,输入部分
        self.need_input()
        # Show part,展示部分
        self.show_init()

        # Set the joint buttons 设置joint按钮
        tk.Button(self.frmLT, text="设置", width=5, command=self.get_joint_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        # coordination settings button,coordination 设置按钮
        tk.Button(self.frmRT, text="设置", width=5, command=self.get_coord_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        # Gripper switch button,夹爪开关按钮
        tk.Button(self.frmLB, text="夹爪(开)", command=self.gripper_open, width=5).grid(
            row=1, column=0, sticky="w", padx=3, pady=50
        )
        tk.Button(self.frmLB, text="夹爪(关)", command=self.gripper_close, width=5).grid(
            row=1, column=1, sticky="w", padx=3, pady=2
        )

    def connect_ser(self):
        rospy.init_node("simple_gui", anonymous=True, disable_signals=True)

        rospy.wait_for_service("get_joint_angles")
        rospy.wait_for_service("set_joint_angles")
        rospy.wait_for_service("get_joint_coords")
        rospy.wait_for_service("set_joint_coords")
        rospy.wait_for_service("switch_gripper_status")
        try:
            self.get_coords = rospy.ServiceProxy("get_joint_coords", GetCoords)
            self.set_coords = rospy.ServiceProxy("set_joint_coords", SetCoords)
            self.get_angles = rospy.ServiceProxy("get_joint_angles", GetAngles)
            self.set_angles = rospy.ServiceProxy("set_joint_angles", SetAngles)
            self.switch_gripper = rospy.ServiceProxy(
                "switch_gripper_status", GripperStatus
            )
        except:
            print("start error ...")
            exit(1)

        print("Connect service success.")

    def set_layout(self):
        self.frmLT = tk.Frame(width=200, height=200)
        self.frmLC = tk.Frame(width=200, height=200)
        self.frmLB = tk.Frame(width=200, height=200)
        self.frmRT = tk.Frame(width=200, height=200)
        self.frmLT.grid(row=0, column=0, padx=1, pady=3)
        self.frmLC.grid(row=1, column=0, padx=1, pady=3)
        self.frmLB.grid(row=1, column=1, padx=2, pady=3)
        self.frmRT.grid(row=0, column=1, padx=2, pady=3)

    def need_input(self):
        # input hint,输入提示
        tk.Label(self.frmLT, text="Joint 1 ").grid(row=0)
        tk.Label(self.frmLT, text="Joint 2 ").grid(row=1)  # the second row,第二行
        tk.Label(self.frmLT, text="Joint 3 ").grid(row=2)
        tk.Label(self.frmLT, text="Joint 4 ").grid(row=3)
        tk.Label(self.frmLT, text="Joint 5 ").grid(row=4)
        tk.Label(self.frmLT, text="Joint 6 ").grid(row=5)

        tk.Label(self.frmRT, text=" x ").grid(row=0)
        tk.Label(self.frmRT, text=" y ").grid(row=1)  # the second row,第二行
        tk.Label(self.frmRT, text=" z ").grid(row=2)
        tk.Label(self.frmRT, text=" rx ").grid(row=3)
        tk.Label(self.frmRT, text=" ry ").grid(row=4)
        tk.Label(self.frmRT, text=" rz ").grid(row=5)

        # Set the default value of the input box,设置输入框的默认值
        self.j1_default = tk.StringVar()
        self.j1_default.set(self.res_angles[0])
        self.j2_default = tk.StringVar()
        self.j2_default.set(self.res_angles[1])
        self.j3_default = tk.StringVar()
        self.j3_default.set(self.res_angles[2])
        self.j4_default = tk.StringVar()
        self.j4_default.set(self.res_angles[3])
        self.j5_default = tk.StringVar()
        self.j5_default.set(self.res_angles[4])
        self.j6_default = tk.StringVar()
        self.j6_default.set(self.res_angles[5])

        self.x_default = tk.StringVar()
        self.x_default.set(self.record_coords[0])
        self.y_default = tk.StringVar()
        self.y_default.set(self.record_coords[1])
        self.z_default = tk.StringVar()
        self.z_default.set(self.record_coords[2])
        self.rx_default = tk.StringVar()
        self.rx_default.set(self.record_coords[3])
        self.ry_default = tk.StringVar()
        self.ry_default.set(self.record_coords[4])
        self.rz_default = tk.StringVar()
        self.rz_default.set(self.record_coords[5])

        # joint input box,joint 输入框
        self.J_1 = tk.Entry(self.frmLT, textvariable=self.j1_default)
        self.J_1.grid(row=0, column=1, pady=3)
        self.J_2 = tk.Entry(self.frmLT, textvariable=self.j2_default)
        self.J_2.grid(row=1, column=1, pady=3)
        self.J_3 = tk.Entry(self.frmLT, textvariable=self.j3_default)
        self.J_3.grid(row=2, column=1, pady=3)
        self.J_4 = tk.Entry(self.frmLT, textvariable=self.j4_default)
        self.J_4.grid(row=3, column=1, pady=3)
        self.J_5 = tk.Entry(self.frmLT, textvariable=self.j5_default)
        self.J_5.grid(row=4, column=1, pady=3)
        self.J_6 = tk.Entry(self.frmLT, textvariable=self.j6_default)
        self.J_6.grid(row=5, column=1, pady=3)

        # coord input box,coord 输入框
        self.x = tk.Entry(self.frmRT, textvariable=self.x_default)
        self.x.grid(row=0, column=1, pady=3, padx=0)
        self.y = tk.Entry(self.frmRT, textvariable=self.y_default)
        self.y.grid(row=1, column=1, pady=3)
        self.z = tk.Entry(self.frmRT, textvariable=self.z_default)
        self.z.grid(row=2, column=1, pady=3)
        self.rx = tk.Entry(self.frmRT, textvariable=self.rx_default)
        self.rx.grid(row=3, column=1, pady=3)
        self.ry = tk.Entry(self.frmRT, textvariable=self.ry_default)
        self.ry.grid(row=4, column=1, pady=3)
        self.rz = tk.Entry(self.frmRT, textvariable=self.rz_default)
        self.rz.grid(row=5, column=1, pady=3)

        # All input boxes, used to get the input data,所有输入框，用于拿输入的数据
        self.all_j = [self.J_1, self.J_2, self.J_3, self.J_4, self.J_5, self.J_6]
        self.all_c = [self.x, self.y, self.z, self.rx, self.ry, self.rz]

        # speed input box,速度输入框
        tk.Label(
            self.frmLB,
            text="speed",
        ).grid(row=0, column=0)
        self.get_speed = tk.Entry(self.frmLB, textvariable=self.speed_d, width=10)
        self.get_speed.grid(row=0, column=1)

    def show_init(self):
        # show,显示
        tk.Label(self.frmLC, text="Joint 1 ").grid(row=0)
        tk.Label(self.frmLC, text="Joint 2 ").grid(row=1)  # the second row,第二行
        tk.Label(self.frmLC, text="Joint 3 ").grid(row=2)
        tk.Label(self.frmLC, text="Joint 4 ").grid(row=3)
        tk.Label(self.frmLC, text="Joint 5 ").grid(row=4)
        tk.Label(self.frmLC, text="Joint 6 ").grid(row=5)

        # get数据

        # show,展示出来
        self.cont_1 = tk.StringVar(self.frmLC)
        self.cont_1.set(str(self.res_angles[0]) + "°")
        self.cont_2 = tk.StringVar(self.frmLC)
        self.cont_2.set(str(self.res_angles[1]) + "°")
        self.cont_3 = tk.StringVar(self.frmLC)
        self.cont_3.set(str(self.res_angles[2]) + "°")
        self.cont_4 = tk.StringVar(self.frmLC)
        self.cont_4.set(str(self.res_angles[3]) + "°")
        self.cont_5 = tk.StringVar(self.frmLC)
        self.cont_5.set(str(self.res_angles[4]) + "°")
        self.cont_6 = tk.StringVar(self.frmLC)
        self.cont_6.set(str(self.res_angles[5]) + "°")
        self.cont_all = [
            self.cont_1,
            self.cont_2,
            self.cont_3,
            self.cont_4,
            self.cont_5,
            self.cont_6,
            self.speed,
            self.model,
        ]

        self.show_j1 = tk.Label(
            self.frmLC,
            textvariable=self.cont_1,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=0, column=1, padx=0, pady=5)

        self.show_j2 = tk.Label(
            self.frmLC,
            textvariable=self.cont_2,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=1, column=1, padx=0, pady=5)
        self.show_j3 = tk.Label(
            self.frmLC,
            textvariable=self.cont_3,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=2, column=1, padx=0, pady=5)
        self.show_j4 = tk.Label(
            self.frmLC,
            textvariable=self.cont_4,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=3, column=1, padx=0, pady=5)
        self.show_j5 = tk.Label(
            self.frmLC,
            textvariable=self.cont_5,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=4, column=1, padx=0, pady=5)
        self.show_j6 = tk.Label(
            self.frmLC,
            textvariable=self.cont_6,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=5, column=1, padx=5, pady=5)

        self.all_jo = [
            self.show_j1,
            self.show_j2,
            self.show_j3,
            self.show_j4,
            self.show_j5,
            self.show_j6,
        ]

        # show,显示
        tk.Label(self.frmLC, text="  x ").grid(row=0, column=3)
        tk.Label(self.frmLC, text="  y ").grid(row=1, column=3)  
        tk.Label(self.frmLC, text="  z ").grid(row=2, column=3)
        tk.Label(self.frmLC, text="  rx ").grid(row=3, column=3)
        tk.Label(self.frmLC, text="  ry ").grid(row=4, column=3)
        tk.Label(self.frmLC, text="  rz ").grid(row=5, column=3)
        self.coord_x = tk.StringVar()
        self.coord_x.set(str(self.record_coords[0]))
        self.coord_y = tk.StringVar()
        self.coord_y.set(str(self.record_coords[1]))
        self.coord_z = tk.StringVar()
        self.coord_z.set(str(self.record_coords[2]))
        self.coord_rx = tk.StringVar()
        self.coord_rx.set(str(self.record_coords[3]))
        self.coord_ry = tk.StringVar()
        self.coord_ry.set(str(self.record_coords[4]))
        self.coord_rz = tk.StringVar()
        self.coord_rz.set(str(self.record_coords[5]))

        self.coord_all = [
            self.coord_x,
            self.coord_y,
            self.coord_z,
            self.coord_rx,
            self.coord_ry,
            self.coord_rz,
            self.speed,
            self.model,
        ]

        self.show_x = tk.Label(
            self.frmLC,
            textvariable=self.coord_x,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=0, column=4, padx=5, pady=5)
        self.show_y = tk.Label(
            self.frmLC,
            textvariable=self.coord_y,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=1, column=4, padx=5, pady=5)
        self.show_z = tk.Label(
            self.frmLC,
            textvariable=self.coord_z,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=2, column=4, padx=5, pady=5)
        self.show_rx = tk.Label(
            self.frmLC,
            textvariable=self.coord_rx,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=3, column=4, padx=5, pady=5)
        self.show_ry = tk.Label(
            self.frmLC,
            textvariable=self.coord_ry,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=4, column=4, padx=5, pady=5)
        self.show_rz = tk.Label(
            self.frmLC,
            textvariable=self.coord_rz,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=5, column=4, padx=5, pady=5)

        # mm， Unit show，单位展示
        self.unit = tk.StringVar()
        self.unit.set("mm")
        for i in range(6):
            tk.Label(self.frmLC, textvariable=self.unit, font=("Arial", 9)).grid(
                row=i, column=5
            )

    def gripper_open(self):
        try:
            self.switch_gripper(True)
        except ServiceException:
            # Probably because the method has no return value, the service throws an unhandled error
            # 可能由于该方法没有返回值，服务抛出无法处理的错误
            pass

    def gripper_close(self):
        try:
            self.switch_gripper(False)
        except ServiceException:
            pass

    def get_coord_input(self):
        # Get the data input by coord and send it to the robotic arm
        # 获取 coord 输入的数据，发送给机械臂
        c_value = []
        for i in self.all_c:
            # print(type(i.get()))
            c_value.append(float(i.get()))
        self.speed = (
            int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        )
        c_value.append(self.speed)
        c_value.append(self.model)
        # print(c_value)
        try:
            self.set_coords(*c_value)
        except ServiceException:
            pass
        self.show_j_date(c_value[:-2], "coord")

    def get_joint_input(self):
        # Get the data input by the joint and send it to the robotic arm
        # 获取joint输入的数据，发送给机械臂
        j_value = []
        for i in self.all_j:
            # print(type(i.get()))
            j_value.append(float(i.get()))
        self.speed = (
            int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        )
        j_value.append(self.speed)

        try:
            self.set_angles(*j_value)
        except ServiceException:
            pass
        self.show_j_date(j_value[:-1])
        # return j_value,c_value,speed

    def get_date(self):
        # Take the data of the robotic arm for display.拿机械臂的数据，用于展示
        t = time.time()
        while time.time() - t < 2:
            self.res = self.get_coords()
            if self.res.x > 1:
                break
            time.sleep(0.1)

        t = time.time()
        while time.time() - t < 2:
            self.angles = self.get_angles()
            if self.angles.joint_1 > 1:
                break
            time.sleep(0.1)
        # print(self.angles.joint_1)
        self.record_coords = [
            round(self.res.x, 2),
            round(self.res.y, 2),
            round(self.res.z, 2),
            round(self.res.rx, 2),
            round(self.res.ry, 2),
            round(self.res.rz, 2),
            self.speed,
            self.model,
        ]
        self.res_angles = [
            round(self.angles.joint_1, 2),
            round(self.angles.joint_2, 2),
            round(self.angles.joint_3, 2),
            round(self.angles.joint_4, 2),
            round(self.angles.joint_5, 2),
            round(self.angles.joint_6, 2),
        ]
        # print('coord:',self.record_coords)
        # print('angles:',self.res_angles)

    # def send_input(self,dates):
    def show_j_date(self, date, way=""):
        # Show data,展示数据
        if way == "coord":
            for i, j in zip(date, self.coord_all):
                # print(i)
                j.set(str(i))
        else:
            for i, j in zip(date, self.cont_all):
                j.set(str(i) + "°")

    def run(self):
        while True:
            try:
                self.win.update()
                time.sleep(0.001)
            except tk.TclError as e:
                if "application has been destroyed" in str(e):
                    break
                else:
                    raise


def main():
    window = tk.Tk()
    window.title("mycobot ros GUI")
    Window(window).run()


if __name__ == "__main__":
    main()