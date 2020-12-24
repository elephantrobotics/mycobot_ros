cmake_minimum_required(VERSION 2.8.3)
project(myCobotROS) 
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg serial)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS}) 

catkin_install_python(PROGRAMS 
  scripts/display.py 
  scripts/control_slider.py 
  scripts/control_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



