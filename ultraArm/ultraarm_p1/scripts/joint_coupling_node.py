#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
"""_summary_
The J2–J3 joint coupling node has the following overall structure:

joint_state_publisher_gui
│
▼
/joint_states_raw
│
▼
joint_coupling_node
(J2-J3 constraints)
│
▼
/joint_states
│
├── robot_state_publisher
│
├── RViz
│
└── slider_control.py
(Controls the real robot)
"""
pub = None


def valid_region(a, b):
    """
    a: J2 angle (deg)
    b: J3 angle (deg)
    """
    
    b = b - 90
    if b > 100 and -21 <= a < 50:
        return False
    if -25 <= a < 0:
        cond1 = math.cos(math.radians(-a + b)) - math.sin(math.radians(45 + a)) <= 7/30
        cond2 = abs(math.cos(math.radians(-a + b))) >= 15.4/30
        return cond1 and cond2

    elif 0 <= a <= 50.87:
        return abs(math.cos(math.radians(a - b))) >= 15.4/30

    elif 50.87 < a < 76.72:
        return True

    elif 76.72 <= a <= 85:
        return abs(math.cos(math.radians(a - b))) >= 6.89/30
    
    return False

def callback(msg):

    pos = list(msg.position)

    j2 = round(math.degrees(pos[1]), 2)
    j3 = math.degrees(pos[5]) + 90
    # rospy.logwarn(f'{j2}')
    if abs(j2) < 1e-3:
        j2 = 0

    if abs(j3) < 1e-3:
        j3 = 0
    
    if not valid_region(j2, j3):
        rospy.logwarn("Invalid J2-J3 combination: %.2f %.2f", j2, j3)
        return

    pub.publish(msg)


def main():
    global pub

    rospy.init_node("joint_coupling_node")

    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    rospy.Subscriber("/joint_states_raw", JointState, callback)

    rospy.spin()


if __name__ == "__main__":
    main()