cmake_minimum_required(VERSION 2.8.3)
project(cobotx_a450)
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
  scripts/detect_marker.py
  scripts/following_marker.py
  scripts/follow_and_pump.py
  scripts/simple_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# find_package(OpenCV REQUIRED)
# add_executable(opencv_camera src/opencv_camera)
# target_link_libraries(opencv_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add_executable(camera_display src/camera_display)
# target_link_libraries(camera_display ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
