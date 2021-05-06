cmake_minimum_required(VERSION 2.8.3)
project(mycobot_ros) 
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  genmsg 
  message_generation 
  serial
)

add_message_files(FILES
  Gripper.msg
)

add_service_files(FILES 
  GetAngles.srv
  SetAngles.srv
  GetCoords.srv
  SetCoords.srv
  GripperStatus.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS}) 

catkin_install_python(PROGRAMS 
  scripts/display.py 
  scripts/control_slider.py 
  scripts/control_marker.py
  scripts/mycobot_ros.py
  scripts/listen_real.py
  scripts/teleop_keyboard.py
  scripts/client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



