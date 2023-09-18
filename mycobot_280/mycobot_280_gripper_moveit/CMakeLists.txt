cmake_minimum_required(VERSION 3.1.3)
project(mycobot_280_gripper_moveit)

find_package(catkin REQUIRED
  rospy
  std_msgs
  moveit_msgs
)

catkin_package()

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/sync_plan.py
  scripts/path_planning_and_obstacle_avoidance_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
