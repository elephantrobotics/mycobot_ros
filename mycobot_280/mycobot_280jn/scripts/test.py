# encoding=utf-8
import time
from pymycobot.mycobotsocket import MyCobotSocket
# from pymycobot.mycobot import MyCobot


# ms.send_angles([50,0,0,0,0,0],50)
i=100

# while i>=0:
for i in range(1,256):
    ms=MyCobotSocket('192.168.125.226',9000)

    ms.connect()
    time.sleep(1)
    print('angles:',ms.get_angles())
    time.sleep(1)
    print('coords:',ms.get_coords())
    # i-=1
    # break
# ms.release_all_servos()

# 获取本机IP地址
import socket
# import struct
# import fcntl
 
# def getip(ethname):
 
#     s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
#     return socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0X8915, struct.pack('256s', ethname[:15]))[20:24])
 
# if __name__=='__main__':
 
#     print(getip('enp0s3'))

# hostname=socket.gethostname()
# print(hostname)
# ip=socket.gethostbyname(hostname)
# print(ip)

# for port in  range(9000,9999):
#     print(port)