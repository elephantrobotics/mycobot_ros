
#!/usr/bin/env python3
#!/usr/bin/env python3

"""[summary]
control_slider.py
This ROS node subscribes to JointState messages and sends the corresponding
angles to a Pro450 robotic arm using pymycobot.

It converts radians from JointState into degrees and publishes them
to the robot through the UltraArmP1.

Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyUSB0'
    baud: serial prot baudrate. Defaults is 115200.
    

Author: WangWeiJian
Date: 2025-11-24
"""

import threading
import rospy
from sensor_msgs.msg import JointState

import math
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '4.0.5'

current_verison = pymycobot.__version__
print('Current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'The current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import UltraArmP1

ua = None
latest_angles = None
last_sent_angles = None
target_dirty = False
state_lock = threading.Lock()
speed = 25
command_rate = 5.0
queue_limit = 0
min_angle_delta = 0.2
use_stop_before_send = True
stop_queue_threshold = 10
stop_settle_time = 0.001


def joint_state_to_angles(data):
    """Convert JointState radians into ultraArm P1 joint angles in degrees."""
    positions_by_name = dict(zip(data.name, data.position))
    required_joints = ["J1", "J2", "J3", "J4"]
    missing_joints = [joint for joint in required_joints if joint not in positions_by_name]
    if missing_joints:
        raise KeyError(", ".join(missing_joints))

    joint1 = round(math.degrees(positions_by_name["J1"]), 2)
    joint2 = round(math.degrees(positions_by_name["J2"]), 2)
    joint3 = round(math.degrees(positions_by_name["J3"]), 2) + 90
    joint4 = round(math.degrees(positions_by_name["J4"]), 2)
    angles_list = [round(angle, 2) for angle in [joint1, joint2, joint3, joint4]]
    return angles_list


def angles_changed(new_angles, old_angles):
    """Return True when the target changed enough to justify a new command."""
    if old_angles is None:
        return True
    return any(abs(new - old) >= min_angle_delta for new, old in zip(new_angles, old_angles))


def get_robot_queue_size():
    """Read firmware queue size when supported by the pymycobot driver."""
    if not hasattr(ua, "get_queue_size"):
        return None
    try:
        size = ua.get_queue_size()
    except Exception as exc:
        rospy.logwarn_throttle(5.0, "Failed to read robot queue size: %s", exc)
        return None
    if isinstance(size, int) and size >= 0:
        return size
    return None


def stop_robot_queue():
    """Stop current motion and clear queued firmware targets when supported."""
    if not hasattr(ua, "stop"):
        rospy.logwarn_throttle(5.0, "Robot driver does not provide stop(); cannot clear queued targets.")
        return False
    try:
        ua.stop()
        if stop_settle_time > 0:
            rospy.sleep(stop_settle_time)
        return True
    except Exception as exc:
        rospy.logwarn_throttle(2.0, "Failed to stop robot before sending latest target: %s", exc)
        return False


def command_worker():
    """Send only the newest slider target at a bounded rate."""
    global last_sent_angles, target_dirty

    rate = rospy.Rate(command_rate)
    while not rospy.is_shutdown():
        with state_lock:
            target = list(latest_angles) if latest_angles is not None else None
            should_send = target_dirty and target is not None and angles_changed(target, last_sent_angles)

        if should_send:
            queue_size = get_robot_queue_size()
            if use_stop_before_send:
                if queue_size is None or queue_size > stop_queue_threshold:
                    if queue_size is not None:
                        rospy.loginfo("clear robot queue before latest target, queue_size: %s", queue_size)
                    stop_robot_queue()
            elif queue_size is not None and queue_size > queue_limit:
                rospy.logwarn_throttle(
                    2.0,
                    "Robot command queue is high (%s > %s); holding newest target.",
                    queue_size,
                    queue_limit,
                )
                rate.sleep()
                continue

            rospy.loginfo("send angles: %s", target)
            try:
                ua.set_angles(target, speed, _async=False)
                with state_lock:
                    last_sent_angles = target
                    if latest_angles == target:
                        target_dirty = False
            except Exception as exc:
                rospy.logerr_throttle(2.0, "Failed to send angles: %s", exc)

        rate.sleep()


def callback(data):
    """Callback function for ROS JointState subscription.

    This function converts incoming joint positions (radians) to angles
    in degrees and sends them to the ultarArm P1 robotic arm.

    Args:
        data (JointState): Joint state message containing joint positions.
    """
    global latest_angles, target_dirty

    try:
        angles_list = joint_state_to_angles(data)
    except KeyError as exc:
        rospy.logwarn_throttle(2.0, "JointState missing required joints: %s", exc)
        return

    with state_lock:
        latest_angles = angles_list
        target_dirty = True


def listener():
    global ua, speed, command_rate, queue_limit, min_angle_delta
    global use_stop_before_send, stop_queue_threshold, stop_settle_time
    rospy.init_node("control_slider", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyUSB0") # Select connected device. 选择连接设备
    baud = rospy.get_param("~baud", 1000000)
    speed = int(rospy.get_param("~speed", 25))
    command_rate = float(rospy.get_param("~command_rate", 5.0))
    queue_limit = int(rospy.get_param("~queue_limit", 0))
    min_angle_delta = float(rospy.get_param("~min_angle_delta", 0.2))
    use_stop_before_send = bool(rospy.get_param("~use_stop_before_send", True))
    stop_queue_threshold = int(rospy.get_param("~stop_queue_threshold", 10))
    stop_settle_time = float(rospy.get_param("~stop_settle_time", 0.02))
    print(port, baud)
    ua = UltraArmP1(port, baud)
    ua.set_joint_enable(0)
    
    threading.Thread(target=command_worker, daemon=True).start()
    rospy.Subscriber("joint_states", JointState, callback, queue_size=1, tcp_nodelay=True)
    
    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()