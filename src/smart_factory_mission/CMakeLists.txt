cmake_minimum_required(VERSION 3.0.2)
project(smart_factory_mission)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  rospy
  smart_factory_interfaces
  smart_factory_manipulation
  smart_factory_navigation
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    rospy
    smart_factory_interfaces
    smart_factory_manipulation
    smart_factory_navigation
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
