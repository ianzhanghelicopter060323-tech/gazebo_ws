cmake_minimum_required(VERSION 3.0.2)
project(smart_factory_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

install(
  PROGRAMS scripts/imgsave scripts/ocr_image scripts/ocr_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
