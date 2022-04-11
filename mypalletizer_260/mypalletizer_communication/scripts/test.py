from pymycobot.mypalletizer import MyPalletizer
mp = MyPalletizer("/dev/ttyUSB0", 115200)
n = mp.get_angles()
# while(1):
#     print(mp.get_angles())
    # print(mp.get_radians())
mp.send_angles([100,0,0,150],30)