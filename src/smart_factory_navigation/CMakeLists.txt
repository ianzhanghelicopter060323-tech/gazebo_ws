cmake_minimum_required(VERSION 3.0.2)
project(smart_factory_navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  move_base_msgs
  nav_msgs
  rospy
  std_msgs
  std_srvs
  tf2_ros
)

catkin_python_setup()

add_action_files(
  FILES
  Navigate.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    message_runtime
    move_base_msgs
    nav_msgs
    rospy
    std_msgs
    std_srvs
    tf2_ros
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

catkin_install_python(
  PROGRAMS scripts/navigation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY action config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
