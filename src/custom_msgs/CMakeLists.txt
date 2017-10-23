
################################
### Auto-generated CMakeList ###
################################
cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)
set( CMAKE_CXX_STANDARD 11 )

################
### Packages ###
################
find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
)

################
### Messages ###
################
add_message_files(
	FILES
	Complex.msg
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
