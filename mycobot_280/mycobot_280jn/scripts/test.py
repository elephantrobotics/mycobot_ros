import time
from pymycobot.mycobotsocket import MyCobotSocket
# from pymycobot.mycobot import MyCobot
ms=MyCobotSocket('192.168.125.226',9000)

ms.connect(serialport="/dev/ttyTHS1", baudrate="1000000")

ms.send_angles([50,0,0,0,0,0],50)
time.sleep(2)

print(ms.get_angles())
ms.release_all_servos()