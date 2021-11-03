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

    def gpiod(self):
    import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(20, GPIO.OUT)
    GPIO.setup(21, GPIO.OUT)
    GPIO.output(20, 0)
    GPIO.output(21, 0)
    time.sleep(3)
    print "close"
    GPIO.output(20, 1)
    GPIO.output(21, 1)


if __name__ == "__main__":
    pump = Pump()
    pump.gpiod()
