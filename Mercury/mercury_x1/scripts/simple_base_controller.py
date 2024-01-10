#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from python_qt_binding.QtWidgets import QPushButton, QWidget, QVBoxLayout, QApplication

class BaseControlPlugin(QWidget):

    def __init__(self):
        super(BaseControlPlugin, self).__init__()
        self.init_ui()
        self.init_ros()

    def init_ui(self):
        self.setWindowTitle("Base Control Plugin")
        layout = QVBoxLayout()

        # 创建控制按钮
        self.move_forward_button = QPushButton("Move Forward")
        self.move_forward_button.clicked.connect(self.move_forward)
        layout.addWidget(self.move_forward_button)

        self.move_backward_button = QPushButton("Move Backward")
        self.move_backward_button.clicked.connect(self.move_backward)
        layout.addWidget(self.move_backward_button)

        self.setLayout(layout)

    def init_ros(self):
        rospy.init_node('base_control_plugin', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def move_base(self, linear_velocity, angular_velocity):
        # 创建一个Twist消息来控制底盘小车
        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity
        print('3')
        # 发布速度消息
        self.velocity_publisher.publish(vel_msg)

    def move_forward(self):
        self.move_base(0.5, 0.0)
        print('1')

    def move_backward(self):
        self.move_base(-0.5, 0.0)
        print('2')

if __name__ == '__main__':
    app = QApplication([])
    base_control_plugin = BaseControlPlugin()
    base_control_plugin.show()
    app.exec_()
