cmake_minimum_required(VERSION 2.8.3)

project(forgemate)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  joint_state_publisher
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  urdf
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


catkin_install_python(PROGRAMS
   scripts/run.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
