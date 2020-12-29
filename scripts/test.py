#!/usr/bin/env python3
from pythonAPI.mycobot import MyCobot

if __name__ == '__main__':
    mycobot = MyCobot()
    mycobot.set_color("ff0000")
    print(mycobot.get_angles_of_radian())
    
