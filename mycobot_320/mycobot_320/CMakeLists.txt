cmake_minimum_required(VERSION 3.0.2)
project(mycobot_320)
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  image_transport
  cv_bridge

  mycobot_communication
  mycobot_description
  
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS std_msgs actionlib

  # INCLUDE_DIRS include
  # LIBRARIES mycobot_320
  # CATKIN_DEPENDS mycobot_communication mycobot_description
  # DEPENDS system_lib
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/mycobot_320_follow_display.py
  scripts/mycobot_320_slider.py
  scripts/mycobot_320_gripper_slider.py
  scripts/mycobot_320_teleop_keyboard.py
  scripts/mycobot_320_listen_real.py
  scripts/mycobot_320_listen_real_of_topic.py
  scripts/mycobot_320_detect_marker.py
  scripts/mycobot_320_following_marker.py
  scripts/mycobot_320_follow_and_pump.py
  scripts/mycobot_320_simple_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# find_package(OpenCV REQUIRED)
# add_executable(opencv_camera src/opencv_camera.cpp)
# Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/opencv_camera.cpp
# )
# target_link_libraries(opencv_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add_executable(camera_display src/camera_display)
# target_link_libraries(camera_display ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
