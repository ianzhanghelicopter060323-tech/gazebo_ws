cmake_minimum_required(VERSION 3.0.2)
project(smart_factory_adaptive_teb)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  teb_local_planner
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smart_factory_adaptive_teb
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    teb_local_planner
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(smart_factory_adaptive_teb
  src/adaptive_teb_local_planner.cpp
  src/scan_analyzer.cpp
)
target_link_libraries(smart_factory_adaptive_teb ${catkin_LIBRARIES})
add_dependencies(smart_factory_adaptive_teb ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_scan_analyzer test/test_scan_analyzer.cpp)
  if(TARGET test_scan_analyzer)
    target_link_libraries(
      test_scan_analyzer smart_factory_adaptive_teb ${catkin_LIBRARIES}
    )
  endif()
endif()

install(
  TARGETS smart_factory_adaptive_teb
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES adaptive_teb_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
