cmake_minimum_required(VERSION 3.0.2)
project(smart_factory_tests)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  gazebo_msgs
  nav_msgs
  rospy
  smart_factory_interfaces
  std_msgs
)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/monitor_navigation_safety.py
  scripts/send_navigation_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
