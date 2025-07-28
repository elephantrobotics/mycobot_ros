from pymycobot import ElephantRobot
import time
if __name__=='__main__':
    # "Connect to robot server"
    elephant_client = ElephantRobot("192.168.1.6", 5001)

    # "Enable TCP communication"
    elephant_client.start_client()

    # "Setting"
    # elephant_client.set_speed(20)   # set speed(20%)
    # elephant_client.set_payload(2.0)   # set payload(2.0Kg)

    # "Power on the robot"
    # elephant_client.power_on()
    
    # "Start the system"
    # elephant_client.state_on()

    # "test move"
    elephant_client.write_coords([-108, -140, 620, 88, -2, -37], 3000)
    elephant_client.wait(3)
    elephant_client.command_wait_done()
