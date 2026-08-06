cmake_minimum_required(VERSION 3.0.2)
project(smart_factory_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  rospy
  sensor_msgs
  smart_factory_interfaces
  tf2_geometry_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    message_filters
    rospy
    sensor_msgs
    smart_factory_interfaces
    tf2_geometry_msgs
    tf2_ros
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

install(
  PROGRAMS
    scripts/cube_locator
    scripts/cube_locator.py
    scripts/imgsave
    scripts/ocr_image
    scripts/ocr_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
