cmake_minimum_required(VERSION 2.8.3)
project(mycobot_280jn)
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  image_transport
  cv_bridge
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS std_msgs actionlib
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/follow_display.py
  scripts/slider_control.py
  scripts/teleop_keyboard.py
  scripts/listen_real.py
  scripts/listen_real_of_topic.py
  scripts/listen_real_of_topic_gripper.py
  scripts/slider_control_gripper.py
  scripts/simple_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


