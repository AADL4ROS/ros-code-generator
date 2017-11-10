
################################
### Auto-generated CMakeList ###
################################
cmake_minimum_required(VERSION 2.8.3)
project(planners)
set( CMAKE_CXX_STANDARD 11 )

################
### Packages ###
################
find_package(catkin REQUIRED COMPONENTS
	roscpp
	map_msgs
	ros_base
	nav_msgs
	geometry_msgs
)

#############
### Build ###
#############
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
	${ros_base_INCLUDE_DIRS}
	include
)
add_executable(global_planner.sbpl src/Sbpl_Planner.cpp)
add_executable(local_planner.dwa src/Dwa_Planner.cpp)
target_link_libraries(global_planner.sbpl
	${catkin_LIBRARIES}
	${ros_base_LIBRARIES}
)
target_link_libraries(local_planner.dwa
	${catkin_LIBRARIES}
	${ros_base_LIBRARIES}
)
