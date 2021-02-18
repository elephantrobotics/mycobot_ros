#!/usr/bin/env python3
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot

class ControlSlider(Node):
    def __init__(self):
        super().__init__('control_slider')
        port = subprocess.check_output(['echo -n /dev/ttyUSB*'], 
                                    shell=True).decode()
        self.mc = MyCobot(port)
        self.sub = self.create_subscription(JointState, "joint_states", self.callback)

    def callback(self, data):
        self.get_logger().info("%s", data.position)
        data_list = []
        for index, value in enumerate(data.position):
            if index != 2:
                value *= -1
            data_list.append(value)

        self.mc.send_radians(data_list, 80)

def main(args=None):
    rclpy.init(args=args)
    try:
        control_slider = ControlSlider()
        rclpy.spin(control_slider)
    finally:
        control_slider.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()