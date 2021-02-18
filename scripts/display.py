#!/usr/bin/env python3
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from pymycobot.mycobot import MyCobot


class Display(Node):
    def __init__(self):
        super().__init__('display')

        port = subprocess.check_output(['echo -n /dev/ttyUSB*'], 
                                    shell=True).decode()
        self.mycobot = MyCobot(port)

        # pub joint state
        self.joint_state_send = JointState()
        self.joint_state_send.header = Header()

        self.joint_state_send.name = [
                                'joint2_to_joint1', 
                                'joint3_to_joint2', 
                                'joint4_to_joint3', 
                                'joint5_to_joint4', 
                                'joint6_to_joint5', 
                                'joint6output_to_joint6'
                                ]
        self.joint_state_send.velocity = [0.0]
        self.joint_state_send.effort = []

        self.marker_ = Marker()
        self.marker_.header.frame_id = '/joint1'
        self.marker_.ns = 'my_namespace'

        self.timer = self.create_timer(0.033, self.timer_callback)  # 30 hz
        self.pub_joint = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_marker = self.create_publisher(Marker, 'visualization_marker', 10)

    def timer_callback(self):
        self.joint_state_send.header.stamp = self.get_clock().now().to_msg()

        angles = self.mycobot.get_radians()
        data_list = []
        for index, value in enumerate(angles):
            if index != 2:
                value *= -1
            data_list.append(value)

        self.joint_state_send.position = data_list

        self.pub_joint.publish(self.joint_state_send)

        coords = self.mycobot.get_coords()
        self.get_logger().info('{}'.format(coords))

        #marker 
        self.marker_.header.stamp = self.get_clock().now().to_msg()
        self.marker_.type = self.marker_.SPHERE
        self.marker_.action = self.marker_.ADD
        self.marker_.scale.x = 0.04
        self.marker_.scale.y = 0.04
        self.marker_.scale.z = 0.04

        #marker position initial 
        # print(coords)
        if not coords:
            coords = [0,0,0,0,0,0]
            self.get_logger().info('error [101]: can not get coord values')

        self.marker_.pose.position.x =  coords[1] / 1000 * -1
        self.marker_.pose.position.y =  coords[0] / 1000 
        self.marker_.pose.position.z =  coords[2] / 1000

        self.marker_.color.a = 1.0
        self.marker_.color.g = 1.0
        self.pub_marker.publish(self.marker_)

def main(args=None):
    rclpy.init(args=args)
    try:
        display = Display()
        rclpy.spin(display)
    finally:
        display.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()