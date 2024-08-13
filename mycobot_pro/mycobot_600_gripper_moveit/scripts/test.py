from pymycobot import ElephantRobot
import time

global mc

mc = ElephantRobot("192.168.1.6",5001)





# 启动机器人必要指令
mc.start_client()
time.sleep(1)
# mc.set_gripper_calibrate()
# time.sleep(1)
# mc.set_gripper_mode(0)
# time.sleep(1)



# mc.power_off()#夹爪透传换IO模式时需要先关闭机器再重启机器人一次，仅使用夹爪透传模式不必关闭机器人
mc.state_off()
time.sleep(3)
mc.power_on()
time.sleep(3)
mc.state_on()
time.sleep(3)
mc.set_gripper_mode(0)

#透传模式

for i in range(3):
    mc.set_gripper_value(10, 20)
    time.sleep(1)
    mc.set_gripper_value(96, 20)
    time.sleep(1)