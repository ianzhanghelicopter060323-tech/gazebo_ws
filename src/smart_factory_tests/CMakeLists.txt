cmake_minimum_required(VERSION 3.0.2)
project(smart_factory_tests)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  rospy
  smart_factory_interfaces
)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/send_navigation_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
