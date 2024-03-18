from pymycobot.mycobot import MyCobot
import time
from pymycobot import PI_BAUD, PI_PORT

mc = MyCobot(PI_PORT, PI_BAUD)

coords = [ 
        [135.0, -65.5, 280.1, 178.99, 5.38, -179.9],
        [136.1, -141.6, 243.9, 178.99, 5.38, -179.9]
    ]

angles = [0, 0, 0, 0, 0, 0]

mc.send_coords(coords[0], 20, 1)
time.sleep(3)
mc.send_coords(coords[1], 20, 1)


