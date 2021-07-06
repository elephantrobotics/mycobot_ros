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
  actionlib
  moveit_msgs
  image_transport
  cv_bridge
)

add_message_files(FILES
  MycobotAngles.msg
  MycobotCoords.msg
  MycobotSetAngles.msg
  MycobotSetCoords.msg
  MycobotGripperStatus.msg
  MycobotPumpStatus.msg
)

add_service_files(FILES 
  GetAngles.srv
  SetAngles.srv
  GetCoords.srv
  SetCoords.srv
  GripperStatus.srv
  PumpStatus.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib moveit_msgs
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}) 

catkin_install_python(PROGRAMS 
  scripts/mycobot_services.py
  scripts/mycobot_topics.py
  scripts/mycobot/follow_display.py 
  scripts/mycobot/slider_control.py 
  scripts/mycobot/teleop_keyboard.py
  scripts/mycobot/listen_real.py
  scripts/mycobot/listen_real_of_topic.py
  scripts/mycobot/detect_marker.py
  scripts/mycobot/following_marker.py
  scripts/mycobot/follow_and_pump.py
  scripts/mycobot/simple_gui.py
  scripts/control_marker.py
  scripts/mycobot_moveit/sync_plan.py
  scripts/client.py
  scripts/slider_600.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(OpenCV REQUIRED)
add_executable(opencv_camera src/opencv_camera)
target_link_libraries(opencv_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(camera_display src/camera_display)
target_link_libraries(camera_display ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
