cmake_minimum_required(VERSION 2.8.3)

project(arm_assem_urdf_v3)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	rospy
	geometry_msgs
	urdf
	xacro
	std_msgs

)

catkin_package(
	CATKIN_DEPENDS
		geometry_msgs
		roscpp
		rospy
		std_msgs
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
