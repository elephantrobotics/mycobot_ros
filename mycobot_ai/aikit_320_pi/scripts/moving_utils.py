#encoding: UTF-8
#!/usr/bin/env python3
import rospy
import time,os

from mycobot_communication.msg import MycobotSetAngles, MycobotSetCoords, MycobotPumpStatus


class Movement(object):
	"""Tools class: Communication with mycobot."""
	def __init__(self):
		super(Movement, self).__init__()
		self.angle_pub = rospy.Publisher("mycobot/angles_goal", MycobotSetAngles, queue_size=5)
		self.coord_pub = rospy.Publisher("mycobot/coords_goal", MycobotSetCoords, queue_size=5)

		self.pump_pub = rospy.Publisher("mycobot/pump_status", MycobotPumpStatus, queue_size=10)

		self.angles = MycobotSetAngles()
		self.coords = MycobotSetCoords()
		self.pump = MycobotPumpStatus()

	def pub_coords(self, item, sp=20, m=1):
		self.coords.x = item[0]
		self.coords.y = item[1]
		self.coords.z = item[2]
		self.coords.rx = item[3]
		self.coords.ry = item[4]
		self.coords.rz = item[5]
		self.coords.speed = sp
		self.coords.model = m
		self.coord_pub.publish(self.coords)


	def pub_angles(self, item, sp):
		self.angles.joint_1 = item[0]
		self.angles.joint_2 = item[1]
		self.angles.joint_3 = item[2]
		self.angles.joint_4 = item[3]
		self.angles.joint_5 = item[4]
		self.angles.joint_6 = item[5]
		self.angles.speed = sp
		self.angle_pub.publish(self.angles)


	def pub_pump(self, flag,Pin):
		self.pump.Status = flag
		self.pump.Pin1 = Pin[0]
		self.pump.Pin2 = Pin[1]
		self.pump_pub.publish(self.pump)
