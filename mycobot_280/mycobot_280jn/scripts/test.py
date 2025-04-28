# encoding=utf-8
import time
import pymycobot
from packaging import version
# min low version require
MAX_REQUIRE_VERSION = '3.6.1'
current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot280socket import MyCobot280Socket
    # from pymycobot.mycobot280 import MyCobot280


# ms.send_angles([50,0,0,0,0,0],50)
i=100

# while i>=0:
for i in range(1,256):
    ms=MyCobot280Socket('192.168.125.226',9000)

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