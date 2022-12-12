#!/usr/bin/env python3
# encoding:utf-8

from tkinter import ttk
from tkinter import *
import os,sys
import time
import subprocess

import threading
from multiprocessing import Process


class Application(object):
    def __init__(self):
        self.win = Tk()
        # 窗口置顶
        self.win.wm_attributes('-topmost', 1)
        self.ros = False
        # 运行的文件
        self.run_py = ""
        # 判断通信口并给权限
        try:
            self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
            self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
            self.robot_raspi = os.popen("ls /dev/ttyAMA*").readline()[:-1]
            self.robot_jes = os.popen("ls /dev/ttyTHS1").readline()[:-1]
            if "dev" in self.robot_wio:
                self.set_file(self.robot_wio)
            elif "dev" in self.robot_m5:
               self.set_file(self.robot_m5)
            elif "dev" in self.robot_raspi:
                self.change_file(self.robot_raspi)
            elif "dev" in self.robot_jes:
                self.change_file(self.robot_jes)
        except Exception as e:
            pass

        # 设置标题
        self.win.title("aikit启动工具")
        self.win.geometry(
            "800x600+100+100")  # 290 160为窗口大小，+10 +10 定义窗口弹出时的默认展示位置
        # 打开ros按钮
        self.btn = Button(self.win, text="open ROS", font=("Helvetica","13"), command=self.open_ros)
        self.btn.grid(row=0)

        self.chanse_code = Label(self.win, text="选择程序:", font=("Helvetica","13"), width=10)
        self.chanse_code.grid(row=1)

        self.myComboList = [u"颜色识别", u"物体识别", u"二维码识别"]
        self.myCombox = ttk.Combobox(self.win, font=("Helvetica","13"), values=self.myComboList)
        self.myCombox.grid(row=1, column=1)

        self.add_btn = Button(self.win, text="添加新的物体图像", font=("Helvetica","13"), command=self.add_img)
        self.add_btn.grid(row=1, column=2)

        self.tips = "1、首先打开ros,大概需要等待5s\n2、选择所要运行的程序点击运行即可,开启大概需要10秒,可以通过查看终端查看开启情况。\n\n添加新的图像：\n1、点击按钮，等待开启摄像头\n2、选中图像框，按z键拍照\n3、使用鼠标框出需要识别的图像区域\n4、按Enter键提取图像\n5、根据终端提示，输入数字(1~4)保存到相对应图像的文件夹，按下Enter键即可保存至对应文件夹。"

        self.btn = Button(self.win, text="运行", font=("Helvetica","13"), command=self.start_run)
        self.btn.grid(row=5)

        self.close = Button(self.win, text="close", font=("Helvetica","13"), command=self.close_py)
        self.close.grid(row=5, column=1)

        self.t2 = None
        self.log_data = Text(self.win, width=74, height=20, font=("Helvetica","13"))
        self.log_data.grid(row=16, column=0, columnspan=10)
        self.log_data.insert(END, self.tips)

        # self.open_ros()
        self.win.protocol('WM_DELETE_WINDOW', self.close_rviz)

    def close_rviz(self):
        os.system(
            "ps -ef | grep -E mycobot.rviz | grep -v 'grep' | awk '{print $2}' | xargs kill -9")
        sys.exit(0)
        
    def set_file(self,port):
        self.command = '<arg name="port" default="{}" />'.format(
            port)
        # 根据通信口修改ros启动文件
        os.system(
            "sed -i '2c {}' ~/catkin_ws/src/mycobot_ros/mycobot_ai/ai_mycobot_280/launch/vision_wio.launch"
            .format(self.command))

    def change_file(self, port):
        command1 = '<arg name="port" default="{}" />'.format(port)
        command2 = '<arg name="baud" default="{}" />'.format(1000000)
        # 根据通信口修改ros启动文件
        os.system(
            "sed -i '2c {}' ~/catkin_ws/src/mycobot_ros/mycobot_ai/launch/vision.launch".format(command1))
        os.system(
            "sed -i '3c {}' ~/catkin_ws/src/mycobot_ros/mycobot_ai/launch/vision.launch".format(command2))

    def start_run(self):
        try:
            print(u"开始运行")
            one = self.myCombox.get()
            if one == u"颜色识别":
                self.run_py = "combine_detect_obj_color.py"
                t2 = threading.Thread(target=self.open_py1)
                t2.setDaemon(True)
                t2.start()
            elif one == u"物体识别":
                self.run_py = "combine_detect_obj_img_folder_opt.py"
                t3 = threading.Thread(target=self.open_py)
                t3.setDaemon(True)
                t3.start()
            elif one == u"二维码识别":
                self.run_py = "detect_encode.py"
                t3 = threading.Thread(target=self.open_py2)
                t3.setDaemon(True)
                t3.start()
        except Exception as e:
            self.tips = str(e)
            self.log_data.insert(END, self.tips)

    def open_py(self):
        os.system(
            "cd /home/h/catkin_ws/src/mycobot_ros/mycobot_ai/ai_mycobot_280 && python scripts/combine_detect_obj_img_folder_opt.py"
        )

    def open_py1(self):
        os.system(
            "python ~/catkin_ws/src/mycobot_ros/mycobot_ai/ai_mycobot_280/scripts/combine_detect_obj_color.py"
        )

    def open_py2(self):
        os.system(
            "python ~/catkin_ws/src/mycobot_ros/mycobot_ai/ai_mycobot_280/scripts/detect_encode.py"
        )

    def add_img(self):
        os.system(
            "python ~/catkin_ws/src/mycobot_ros/mycobot_ai/ai_mycobot_280/scripts/add_img.py"
        )

    def open_ros(self):
        if self.ros:
            print("ros is opened")
            return
        # t1 = threading.Thread(target=self.ross)
        # t1.setDaemon(True)
        # t1.start()
        self.ross()
        self.ros = True

    def ross(self):
        # os.system(
        #     "roslaunch ~/catkin_ws/src/mycobot_ros/mycobot_ai/ai_mycobot_280/launch/vision_wio.launch"
        # )
        p = subprocess.Popen(["roslaunch", "/home/h/catkin_ws/src/mycobot_ros/mycobot_ai/ai_mycobot_280/launch/vision_wio.launch"],shell=False, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    def close_py(self):
        t1 = threading.Thread(target=self.close_p)
        t1.setDaemon(True)
        t1.start()

    def close_p(self):
        # 关闭ai程序
        os.system("ps -ef | grep -E " + self.run_py +
                  " | grep -v 'grep' | awk '{print $2}' | xargs kill -9")

    def get_current_time(self):
        # 日志时间
        """Get current time with format."""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S",
                                     time.localtime(time.time()))
        return current_time

    def write_log_to_Text(self, logmsg):
        # 设置日志函数
        global LOG_NUM
        current_time = self.get_current_time()
        logmsg_in = str(current_time) + " " + str(logmsg) + "\n"  # 换行

        if LOG_NUM <= 18:
            self.log_data_Text.insert(END, logmsg_in)
            LOG_NUM += len(logmsg_in.split("\n"))
            # print(LOG_NUM)
        else:
            self.log_data_Text.insert(END, logmsg_in)
            self.log_data_Text.yview("end")

    def run(self):
        self.win.mainloop()


if __name__ == "__main__":
    mc = Application()
    mc.run()
