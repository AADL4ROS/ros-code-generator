
################################
### Auto-generated CMakeList ###
################################
cmake_minimum_required(VERSION 2.8.3)
project(custom_srvs)
set( CMAKE_CXX_STANDARD 11 )

################
### Packages ###
################
find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	geometry_msgs
)

################
### Services ###
################
add_service_files(
	FILES
	ServiceA.srv
)

##################################
### MSGS and SRVS Dependencies ###
##################################
generate_messages(
	DEPENDENCIES
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
