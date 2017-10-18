
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
	roscpp
	std_msgs
	message_generation
)

################
### Messages ###
################
add_message_files(
	FILES
	Complex.msg
}

##################################
### MSGS and SRVS Dependencies ###
##################################
generate_messages(
	DEPENDENCIES
	std_msgs
)

#############
### Build ###
#############
include_directories(
	${catkin_INCLUDE_DIRS}
	${ros_base_INCLUDE_DIRS}
	include
)
