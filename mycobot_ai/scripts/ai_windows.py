#!/usr/bin/env python3
# encoding:utf-8

from tkinter import ttk
from tkinter import *
import os
import time

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
            self.ports = os.popen("ls /dev/ttyUSB*").readline()
            if "dev" not in self.ports:
                self.ports = os.popen("ls /dev/ttyACM*").readline()[:-1]
            self.command = '<arg name="port" default="{}" />'.format(
                self.ports)
            # 根据通信口修改ros启动文件
            os.system(
                "sed -i '2c {}' /home/h/catkin_mycobot/src/mycobot_ros/mycobot_ai/launch/vision.launch"
                .format(self.command))
        except Exception as e:
            pass

        # 设置标题
        self.win.title("aikit启动工具")
        self.win.geometry(
            "550x350+100+100")  # 290 160为窗口大小，+10 +10 定义窗口弹出时的默认展示位置
        # 打开ros按钮
        self.btn = Button(self.win, text="open ROS", command=self.open_ros)
        self.btn.grid(row=0)

        self.chanse_code = Label(self.win, text="选择程序:", width=10)
        self.chanse_code.grid(row=1)

        self.myComboList = [u"颜色识别", u"物体识别", u"二维码识别"]
        self.myCombox = ttk.Combobox(self.win, values=self.myComboList)
        self.myCombox.grid(row=1, column=1)

        self.add_btn = Button(self.win, text="添加新的物体图像", command=self.add_img)
        self.add_btn.grid(row=1, column=2)

        # self.set_xy = Label(self.win, text="set_xy:", width=10)
        # self.set_xy.grid(row=1)

        # self.x = Label(self.win, text="x:")
        # self.x.grid(row=2)
        # self.v1 = StringVar()
        # self.e1 = Entry(self.win,textvariable=self.v1, width=10)
        # self.e1.insert(0,0)
        # self.e1.grid(row=2,column=1)

        # self.y = Label(self.win, text="y:")
        # self.y.grid(row=3)
        # self.v2 = StringVar()
        # self.e2 = Entry(self.win,textvariable=self.v2, width=10)
        # self.e2.insert(0,0)
        # self.e2.grid(row=3,column=1)
        self.tips = "1、首先打开ros,大概需要等待15s\n2、选择所要运行的程序点击运行即可,开启大概需要10秒,可以通过查看终端查看开启情况。\n\n添加新的图像：\n1、点击按钮，等待开启摄像头\n2、选中图像框，按z键拍照\n3、使用鼠标框出需要识别的图像区域\n4、按Enter键提取图像\n5、再次按Enter键保存即可"

        self.btn = Button(self.win, text="运行", command=self.start_run)
        self.btn.grid(row=5)

        self.close = Button(self.win, text="close", command=self.close_py)
        self.close.grid(row=5, column=1)

        self.t2 = None
        self.log_data = Text(self.win, width=66, height=10)
        self.log_data.grid(row=16, column=0, columnspan=10)
        self.log_data.insert(END, self.tips)
        # self.code_list = ttk.Combobox(self.win, width=15)
        # self.code_list["value"] = ("颜色识别", "物体识别", "二维码识别")
        # self.code_list.current(0)
        # self.code_list.grid(row=1, column=1)

    def start_run(self):
        try:
            print(u"开始运行")
            one = self.myCombox.get()
            if one == u"颜色识别":
                self.run_py = "detect_obj_color.py"
                t2 = threading.Thread(target=self.open_py1)
                t2.setDaemon(True)
                t2.start()
            elif one == u"物体识别":
                self.run_py = "detect_obj_img.py"
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
            "python /home/h/catkin_mycobot/src/mycobot_ros/mycobot_ai/scripts/detect_obj_img.py"
        )

    def open_py1(self):
        os.system(
            "python /home/h/catkin_mycobot/src/mycobot_ros/mycobot_ai/scripts/detect_obj_color.py"
        )

    def open_py2(self):
        os.system(
            "python /home/h/catkin_mycobot/src/mycobot_ros/mycobot_ai/scripts/detect_encode.py"
        )

    def add_img(self):
        os.system(
            "python /home/h/catkin_mycobot/src/mycobot_ros/mycobot_ai/scripts/add_img.py"
        )

    def open_ros(self):
        if self.ros:
            print("ros is opened")
            return
        t1 = threading.Thread(target=self.ross)
        t1.setDaemon(True)
        t1.start()
        self.ros = True

    def ross(self):
        os.system(
            "roslaunch ~/catkin_mycobot/src/mycobot_ros/mycobot_ai/launch/vision.launch"
        )

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
