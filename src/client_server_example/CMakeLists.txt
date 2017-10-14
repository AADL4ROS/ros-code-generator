
################################
### Auto-generated CMakeList ###
################################
cmake_minimum_required(VERSION 2.8.3)
project(client_server_example)

################
### Packages ###
################
find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	ros_base
)

################
### Services ###
################
add_service_files(
	FILES
	Custom_Service.srv
}

#############
### Build ###
#############
include_directories(
	${catkin_INCLUDE_DIRS}
	${ros_base_INCLUDE_DIRS}
)
add_executable(client.impl src/Client.cpp)
add_executable(server_node.impl src/Server_Node.cpp)
target_link_libraries(client.impl
	${catkin_LIBRARIES}
	${ros_base_LIBRARIES}
)
target_link_libraries(server_node.impl
	${catkin_LIBRARIES}
	${ros_base_LIBRARIES}
)
