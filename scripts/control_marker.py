#!/usr/bin/env python2
# from std_msgs.msg import String
import time, subprocess

import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from pymycobot.mycobot import MyCobot


server = None
menu_handler = MenuHandler()

# center of the body
center_x_changed = 0
center_y_changed = 0
center_z_changed = 0

# marker box 
def processFeedback( feedback ):#
    global center_x_changed, center_y_changed, center_z_changed
    current_x = feedback.pose.position.x
    current_y = feedback.pose.position.y
    current_z = feedback.pose.position.z

    _x = feedback.pose.orientation.x
    _y = feedback.pose.orientation.y
    _z = feedback.pose.orientation.z

    center_x_changed = current_x
    center_y_changed = current_y
    center_z_changed = current_z
    print(center_x_changed, center_y_changed, center_z_changed,
         _x, _y, _z)

    coords = [
                current_y * 1000, 
                current_x * -1000, 
                current_z * 1000, 
                _x * 100,
                _y * 100,
                _z * 100,
             ]
    speed = 80
    mode = 0
    mycobot.send_coords(coords, speed, mode)

    server.applyChanges()

def makeBox( msg ):#
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.1
    marker.scale.y = msg.scale * 0.1
    marker.scale.z = msg.scale * 0.1
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.1

    return marker

def makeBoxControl( msg ):#
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def make6DofMarker(fixed, interaction_mode, position, orientation, show_6dof = False):#
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/joint1"
    int_marker.pose.position = position	# Defined the position of the marker
    int_marker.pose.orientation = orientation
    int_marker.scale = 0.1

    int_marker.name = "simple_6dof"
    int_marker.description = "mycobot_controller"

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
      
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )

# marker box finish

def listener():
    global server
    rospy.init_node('control_marker', anonymous=True)

    coords = mycobot.get_coords()
    # print(coords)

    # crate a timer to update the pushlished transforms
    server = InteractiveMarkerServer('mycobot_controller')
    menu_handler.insert('First Entry', callback=processFeedback)
    menu_handler.insert('Second Entry', callback=processFeedback)

    if not coords:
        coords = [0,0,0,0,0,0]
        rospy.loginfo('error [101]: can not get coord values')	
    # initial position
    position = Point(coords[1] / -1000, coords[0] / 1000, coords[2] / 1000)
    # orientation = Quaternion(coords[4] / 100, coords[3] / 100, coords[5] / 100, 1)
    orientation = Quaternion(0, 0, 0, 1)
    make6DofMarker(True, InteractiveMarkerControl.NONE, 
                   position, orientation, True)
    server.applyChanges()

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(30) # 10hz

    # pub joint state
    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = [
                            'joint2_to_joint1', 
                            'joint3_to_joint2', 
                            'joint4_to_joint3', 
                            'joint5_to_joint4', 
                            'joint6_to_joint5', 
                            'joint6output_to_joint6'
                            ]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    marker_ = Marker()
    marker_.header.frame_id = '/joint1'
    marker_.ns = 'my_namespace'

    while not rospy.is_shutdown():
        joint_state_send.header.stamp = rospy.Time.now()

        angles = mycobot.get_radians()
        rospy.loginfo(angles)
        if angles:
            data_list = []
            for index, value in enumerate(angles):
                if index != 2:
                    value *= -1
                data_list.append(value)

            
            joint_state_send.position = data_list

            pub.publish(joint_state_send)

        rate.sleep()


if __name__ == '__main__':
    port = subprocess.check_output(['echo -n /dev/ttyUSB*'], 
                                    shell=True).decode()
    mycobot = MyCobot(port)
    listener()
