from pymycobot import MyArmM, MyCobot
import time
# myarm_m = MyArmM('/dev/ttyACM1', 1000000, debug=False)
mc = MyCobot('/dev/ttyACM2', 115200, debug=1)
# mc.power_on()
for i in range(1, 7):
    mc.focus_servo(i)
    time.sleep(0.5)