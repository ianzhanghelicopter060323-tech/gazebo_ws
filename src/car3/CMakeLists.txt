cmake_minimum_required(VERSION 2.8.3)

project(car3)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

catkin_install_python(PROGRAMS
  scripts/gripper_mimic.py
  scripts/grasp_attach.py
  scripts/spawn_cubes.py
  scripts/spawn_calibration_cubes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
