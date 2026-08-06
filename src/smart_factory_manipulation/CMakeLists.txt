cmake_minimum_required(VERSION 3.0.2)
project(smart_factory_manipulation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    rospy
    sensor_msgs
    std_msgs
    trajectory_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
