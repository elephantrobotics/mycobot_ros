# encoding: utf-8

import rospy
import time
from visualization_msgs.msg import Marker

class Send_marker(object):
    def __init__(self):
        # 继承object类对象
        super(Send_marker, self).__init__()
        # 初始化一个节点，如果没有创建节点会导致无法发布信息
        rospy.init_node("send_marker", anonymous=True)
        # 创建一个发布者，用来发布marker
        self.pub = rospy.Publisher("/test_marker", Marker, queue_size=1)
        # 创建一个marker用来创建方块模型
        self.marker = Marker()
        # 配置其所属关系，其坐标均是相对于/joint1而言的。
        # /joint1在模型中代表机械臂的底部
        self.marker.header.frame_id = "/joint1"
        # 设置marker的名称
        self.marker.ns = "test_marker"
        # 设置marker的类型是方块
        self.marker.type = self.marker.CUBE
        # 设置marker的动作为添加（没有这个名称的marker就为其添加一个）
        self.marker.action = self.marker.ADD
        # 设置marker的实际大小情况，单位为m
        self.marker.scale.x = 0.04
        self.marker.scale.y = 0.04
        self.marker.scale.z = 0.04
        # 设置marker的颜色，1.0表示255（这表示着一种比率换算）
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.color.r = 1.0
        # 初始化marker的位置以及其四维姿态
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0.03
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0

    # 修改坐标并发布marker
    def pub_marker(self, x, y, z=0.03):
        # 设置marker的时间戳
        self.marker.header.stamp = rospy.Time.now()
        # 设置marker的空间坐标
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        # 发布marker
        self.pub.publish(self.marker)

    # 让marker发生位移效果
    def run(self):
        time.sleep(1)
        self.pub_marker(0.2, 0)
        time.sleep(1)
        self.pub_marker(0.15, -0.05)
        time.sleep(1)
        self.pub_marker(0.15, 0.05)
        time.sleep(1)
        self.pub_marker(0.1, 0)
        time.sleep(1)
        self.pub_marker(0.136, -0.141)
        time.sleep(1)
        self.pub_marker(0.238, -0.147)
        time.sleep(1)

if __name__ == '__main__':
    marker = Send_marker()
    marker.run()
