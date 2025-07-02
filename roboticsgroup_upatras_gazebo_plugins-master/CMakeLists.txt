cmake_minimum_required(VERSION 3.1.0)
project(roboticsgroup_upatras_gazebo_plugins)

find_package(catkin REQUIRED
  COMPONENTS
    control_toolbox
    gazebo_ros
    roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    roboticsgroup_upatras_gazebo_mimic_joint_plugin
    roboticsgroup_upatras_gazebo_disable_link_plugin
  CATKIN_DEPENDS
    control_toolbox
    gazebo_ros
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(roboticsgroup_upatras_gazebo_mimic_joint_plugin src/mimic_joint_plugin.cpp)
target_link_libraries(roboticsgroup_upatras_gazebo_mimic_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(roboticsgroup_upatras_gazebo_disable_link_plugin src/disable_link_plugin.cpp)
target_link_libraries(roboticsgroup_upatras_gazebo_disable_link_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS
    roboticsgroup_upatras_gazebo_mimic_joint_plugin
    roboticsgroup_upatras_gazebo_disable_link_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
