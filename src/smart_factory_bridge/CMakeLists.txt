cmake_minimum_required(VERSION 3.0.2)
project(smart_factory_bridge)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  rospy
  std_msgs
  smart_factory_interfaces
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    actionlib
    rospy
    std_msgs
    smart_factory_interfaces
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

catkin_install_python(
  PROGRAMS scripts/vehicle_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
