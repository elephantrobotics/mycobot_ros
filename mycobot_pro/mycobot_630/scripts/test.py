import time
import sys
from pymycobot.elephantrobot import ElephantRobot

mc = ElephantRobot('192.168.1.159', 5001, debug=True)

res = mc.start_client()
if res != "":
    print('quit')
    sys.exit(1)

print(mc.get_angles())
# mc.write_angle(0, 0, 1000)
