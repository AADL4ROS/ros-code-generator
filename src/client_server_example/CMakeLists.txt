
################################
### Auto-generated CMakeList ###
################################
cmake_minimum_required(VERSION 2.8.3)
project(client_server_example)
set( CMAKE_CXX_STANDARD 11 )

################
### Packages ###
################
find_package(catkin REQUIRED COMPONENTS
	custom_srvs
	std_msgs
	custom_msgs
	roscpp
	ros_base
)

##################################
### MSGS and SRVS Dependencies ###
##################################
generate_messages()

#############
### Build ###
#############
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
	${ros_base_INCLUDE_DIRS}
	include
)
add_executable(talker.impl src/Publisher.cpp)
add_executable(server_node.impl src/Server_Node.cpp)
add_executable(talk_list.impl src/Talk_List_Node.cpp)
target_link_libraries(talker.impl
	${catkin_LIBRARIES}
	${ros_base_LIBRARIES}
)
target_link_libraries(server_node.impl
	${catkin_LIBRARIES}
	${ros_base_LIBRARIES}
)
target_link_libraries(talk_list.impl
	${catkin_LIBRARIES}
	${ros_base_LIBRARIES}
)
