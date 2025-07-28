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
    from pymycobot.mypalletizer260 import MyPalletizer260
mp = MyPalletizer260("/dev/ttyUSB0", 115200)
n = mp.get_angles()
# while(1):
#     print(mp.get_angles())
    # print(mp.get_radians())
mp.send_angles([100,0,0,150],30)