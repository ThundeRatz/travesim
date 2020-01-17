cmake_minimum_required(VERSION 2.8.3)

project(vss_simulation)

find_package(catkin REQUIRED)

catkin_package()

# find_package(?roslaunch)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
