#encoding: UTF-8
#!/usr/bin/env python2

import rospy
import time
from moving_utils import Movement

class Pump(Movement):

	def __init__(self):
		super(Pump, self).__init__()
		rospy.init_node("pump", anonymous=True)

	def run(self):
		self.pub_pump(False)
		time.sleep(1)
		self.pub_pump(True)
		time.sleep(5)
		self.pub_pump(False)

if __name__ == "__main__":
	pump = Pump()
	pump.run()