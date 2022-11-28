#!/usr/bin/python
# -*- coding: UTF-8 -*-

from pymycobot.mycobot import MyCobot
import time
import threading
import os

print(os.path.join(os.path.dirname(__file__)))

mc = MyCobot('/dev/ttyUSB0', 115200)
mc.set_tool_reference([-50,0,0,0,0,0])
mc.set_end_type(1)

init_angles = [0, 0.52, -85.69, 0.0, 89.82, 0.08]
start_angles = [18.64, 0.52, -85.69, 0.0, 89.82, 0.08]
end_angles = [-18.56, 0.52, -85.69, 0.0, 89.82, 0.08]

ang = []

def wait_time(t):
    global ang
    for i in range(t*10+1):
        time.sleep(0.1)
        ang1 = mc.get_angles()
        ang.append(ang1)
        coord = mc.get_coords()
        # print('ange-------->', ang)
        print('coord-------->', coord)
    

def get_ang():
    i = 0
    
    print('55555555555')
    ang_list = None
    while True:
        for j in range(i,len(ang)):
            i+=1
        # print('1111111111111')
        # if ang_list != ang:
        #     ang_list = ang
            
            print('----------', ang[j])

t = threading.Thread(target=get_ang)
t.setDaemon(True)
t.start()

def move():
    try:
        mc.send_angles(init_angles, 5)
        time.sleep(0.1)
        
        # for _ in range(50):
            
        #     mc.send_angles(start_angles, 5)
        #     wait_time(6)
        #     mc.send_angles(end_angles, 5)
        #     wait_time(6)
    except Exception as e:
        print(e)

while True:
    t = wait_time(1)
    print('dddd', t)
    move()

