cmake_minimum_required(VERSION 3.0.2)
project(mycobotpro630_gazebo)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS gazebo_ros_control hardware_interface
  message_generation moveit_core moveit_msgs pluginlib roscpp std_srvs
  transmission_interface urdf)
find_package(gazebo REQUIRED)

add_service_files(FILES Pro630Rpc.srv)
generate_messages()
catkin_package(
  LIBRARIES pro630_current_start_adapter pro630_gazebo_hw
  CATKIN_DEPENDS gazebo_ros_control hardware_interface message_runtime moveit_core
    moveit_msgs pluginlib roscpp std_srvs transmission_interface urdf
)

add_library(pro630_current_start_adapter src/use_current_start_state.cpp)
target_include_directories(pro630_current_start_adapter PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(pro630_current_start_adapter ${catkin_LIBRARIES})
add_dependencies(pro630_current_start_adapter ${catkin_EXPORTED_TARGETS})

add_library(pro630_gazebo_hw src/gazebo_hw.cpp)
target_include_directories(pro630_gazebo_hw PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(pro630_gazebo_hw ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(pro630_gazebo_hw ${catkin_EXPORTED_TARGETS})

# Install adjacent Python modules together so the exact same imports work
# when started with rosrun, roslaunch or by the feature supervisor.
set(nodes
  pro630_bringup.py pro630_auxiliary.py
  pro630_environment_state.py pro630_pose_gate.py pro630_robot_bridge.py
  pro630_collision_scene.py verify_initial_pose.py pro630_control_session.py
  slider_control_gazebo.py pro630_slider_gui.py teleop_keyboard_gazebo.py
  pro630_drag_follow.py pro630_gui_backend.py pro630_control_gui.py
  probe_collision_states.py build_collision_meshes.py export_exact_collision_meshes.py)
foreach(node ${nodes})
  list(APPEND node_files scripts/${node})
endforeach()
catkin_install_python(PROGRAMS ${node_files} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
file(GLOB helper_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "scripts/*.py")
list(REMOVE_ITEM helper_files ${node_files})
foreach(helper ${helper_files})
  get_filename_component(filename "${helper}" NAME)
  configure_file("${helper}"
    "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${filename}" COPYONLY)
endforeach()
install(FILES ${helper_files} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS pro630_current_start_adapter pro630_gazebo_hw
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY config launch meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES planning_request_adapters.xml gazebo_hw_plugins.xml README.md .setup_assistant
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
