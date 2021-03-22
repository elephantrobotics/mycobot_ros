import time, random, subprocess
from pymycobot.mycobot import MyCobot
# from pythonAPI.mycobot3 import MyCobot as MyCobot3
from pymycobot.genre import Angle, Coord

if __name__ == '__main__': 
    port = subprocess.check_output(['echo -n /dev/ttyUSB*'], 
                                    shell=True).decode()
    mycobot = MyCobot(port)

    print('Start check api\n')

    print('::get_angles()')
    print('==> degrees: {}\n'.format(mycobot.get_angles()))
    time.sleep(0.5)

    print('::get_radians()')
    print('==> radians: {}\n'.format(mycobot.get_radians()))
    time.sleep(0.5)

    print('::send_angles()')
    mycobot.send_angles([0,0,0,0,0,0], 80)
    print('==> set angles [0,0,0,0,0,0], speed 80\n')
    print('Is moving: {}'.format(mycobot.is_moving()))
    time.sleep(3)

    print('::send_radians')
    mycobot.send_radians([1,1,1,1,1,1], 70)
    print('==> set raidans [1,1,1,1,1,1], speed 70\n')
    time.sleep(1.5)

    print('::send_angle()')
    mycobot.send_angle(Angle.J2.value, 10, 50)
    print('==> angle: joint2, degree: 10, speed: 50\n')
    time.sleep(1)

    print('::get_coords()')
    print('==> coords {}\n'.format(mycobot.get_coords()))
    time.sleep(0.5)

    print('::send_coords()')
    coord_list = [160, 160, 160, 0, 0, 0]
    mycobot.send_coords(coord_list, 70, 0)
    print('==> send coords [160,160,160,0,0,0], speed 70, mode 0\n')
    time.sleep(3.0)

    print(mycobot.is_in_position(coord_list, 1))
    time.sleep(1)

    print('::send_coord()')
    mycobot.send_coord(Coord.X.value, -40, 70)
    print('==> send coord id: X, coord value: -40, speed: 70\n')
    time.sleep(2)

    print('::set_free_mode()')
    mycobot.set_free_mode()
    print('==> into free moving mode.')
    print('=== check end <==\n')
