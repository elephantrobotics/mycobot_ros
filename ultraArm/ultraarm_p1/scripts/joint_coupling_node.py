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
last_valid_msg = None
last_invalid_pair = None
was_invalid = False
J2_RANGE = (-18.0, 85.0)
J3_RANGE = (-1.0, 110.0)
ZERO_EPS_DEG = 0.1
INVALID_WARN_DELTA_DEG = 0.5


def joint_angle_deg(msg, joint_name):
    try:
        index = msg.name.index(joint_name)
    except ValueError:
        raise KeyError(joint_name)

    return math.degrees(msg.position[index])

def snap_zero(angle_deg):
    return 0.0 if abs(angle_deg) < ZERO_EPS_DEG else angle_deg

def valid_region(a, b):
    """
    a: J2 angle (deg)
    b: J3 angle (deg)
    """

    if not (J2_RANGE[0] <= a <= J2_RANGE[1] and J3_RANGE[0] <= b <= J3_RANGE[1]):
        return False

    if -18 <= a < 0:
        cond1 = math.cos(math.radians(-a + b)) - math.sin(math.radians(45 + a)) <= 7/30
        cond2 = abs(math.cos(math.radians(-a + b))) >= 15.4/30
        return cond1 and cond2

    elif 0 <= a <= 50.87:
        return math.cos(math.radians(a - b)) >= 15.4/30

    elif 50.87 < a < 76.72:
        return True

    elif 76.72 <= a <= 85:
        return abs(math.cos(math.radians(a - b))) >= 6.89/30
    
    return False

def callback(msg):
    global last_valid_msg, last_invalid_pair, was_invalid
    try:
        j2 = round(snap_zero(joint_angle_deg(msg, "J2")), 2)
        j3 = round(snap_zero(joint_angle_deg(msg, "J3")), 2)
    except (KeyError, IndexError):
        rospy.logwarn_throttle(2.0, "JointState missing J2 or J3; message ignored")
        return

    # rospy.logwarn(f'{j2}')
    if not valid_region(j2, j3):
        # rospy.logwarn_throttle(1.0, "Invalid J2-J3 combination: %.2f %.2f", j2, j3)
        should_warn = not was_invalid
        if last_invalid_pair is not None:
            should_warn = should_warn or abs(j2 - last_invalid_pair[0]) >= INVALID_WARN_DELTA_DEG
            should_warn = should_warn or abs(j3 - last_invalid_pair[1]) >= INVALID_WARN_DELTA_DEG
        else:
            should_warn = True

        if should_warn:
            rospy.logwarn("Invalid J2-J3 combination: %.2f %.2f", j2, j3)
            last_invalid_pair = (j2, j3)
        was_invalid = True
        if last_valid_msg is not None:
            safe_msg = JointState()
            safe_msg.header = last_valid_msg.header
            safe_msg.header.stamp = rospy.Time.now()
            safe_msg.name = list(last_valid_msg.name)
            safe_msg.position = list(last_valid_msg.position)
            safe_msg.velocity = list(last_valid_msg.velocity)
            safe_msg.effort = list(last_valid_msg.effort)
            pub.publish(safe_msg)
        return
    
    was_invalid = False
    last_invalid_pair = None
    last_valid_msg = msg
    pub.publish(msg)


def main():
    global pub

    rospy.init_node("joint_coupling_node")

    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    rospy.Subscriber("/joint_states_raw", JointState, callback)

    rospy.spin()


if __name__ == "__main__":
    main()