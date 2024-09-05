import time
from pymycobot.mycobot import MyCobot


from pymycobot import *
import time
import datetime
m = MyCobot('/dev/ttyUSB0', 115200)
time.sleep(2)
delay_time = 0.1
run_delay_time = 1
angles = [0,0,0,0,0,0]
coords = [0,0,0,0,0,0]
angle_loss_count = 0
coord_loss_count = 0
total_count = 0
send_angles = [[0,0,0,0,0,0], [0,0,0,0,20,20]]
sp = 50
# m = MyCobot("com64", 115200)

# time.sleep(2) #open port,need wait
# print(m.get_radians())
while 1:
    for i in range(len(send_angles)) :
        angles = m.get_angles()
        if (angles is None):
            angle_loss_count = angle_loss_count + 1
            #angles = m.get_angles()
        time.sleep(delay_time)
        coords = m.get_coords()
        if (coords is None):
            coord_loss_count = coord_loss_count + 1
            #coords = m.get_coords()
        time.sleep(delay_time)
        m.send_angles(send_angles[i], sp)
        time.sleep(run_delay_time)
        total_count = total_count + 1
        now = datetime.datetime.now()
        print(now, "angles, coords ", angles, coords, angle_loss_count, coord_loss_count, total_count)
        time.sleep(delay_time)