cmake_minimum_required(VERSION 3.0.2)
project(smart_factory_mission)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  sensor_msgs
  smart_factory_interfaces
  std_msgs
  std_srvs
  tf2_ros
  trajectory_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    nav_msgs
    rospy
    sensor_msgs
    smart_factory_interfaces
    std_msgs
    std_srvs
    tf2_ros
    trajectory_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

catkin_install_python(
  PROGRAMS scripts/mission_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
