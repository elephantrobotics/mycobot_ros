cmake_minimum_required(VERSION 3.1.3)
project(ultraarm_p1_moveit)

# find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  moveit_msgs
  visualization_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  pluginlib
)

catkin_package(
  LIBRARIES ultraarm_p1_moveit_coupling_adapter
  CATKIN_DEPENDS moveit_core moveit_ros_planning pluginlib
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(ultraarm_p1_moveit_coupling_adapter SHARED
  src/coupling_planning_request_adapter.cpp
)
target_link_libraries(ultraarm_p1_moveit_coupling_adapter ${catkin_LIBRARIES})
set_target_properties(ultraarm_p1_moveit_coupling_adapter PROPERTIES
  COMPILE_FLAGS "-std=c++11"
)

catkin_install_python(PROGRAMS
  scripts/sync_plan.py
  scripts/mimic_preview_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS ultraarm_p1_moveit_coupling_adapter
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES coupling_adapter_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
