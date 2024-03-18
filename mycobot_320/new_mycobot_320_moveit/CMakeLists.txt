cmake_minimum_required(VERSION 3.0.2)
project(new_mycobot_320_moveit)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


