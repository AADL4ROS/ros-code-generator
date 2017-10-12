
################################
### Auto-generated CMakeList ###
################################
cmake_minimum_required(VERSION 2.8.3)
project(example_node)

################
### Packages ###
################
find_package(catkin REQUIRED COMPONENTS
	std_msgs
	roscpp
	geometry_msgs
	ros_base
)

#############
### Build ###
#############
include_directories(
	${catkin_INCLUDE_DIRS}
	${ros_base_INCLUDE_DIRS}
)
add_executable(talker.impl src/Talker_Node.cpp)
add_executable(listener.impl src/Listener_Node.cpp)
add_executable(talk_list.impl src/Talk_List_Node.cpp)
target_link_libraries(talker.impl
	${catkin_LIBRARIES}
	${ros_base_LIBRARIES}
)
target_link_libraries(listener.impl
	${catkin_LIBRARIES}
	${ros_base_LIBRARIES}
)
target_link_libraries(talk_list.impl
	${catkin_LIBRARIES}
	${ros_base_LIBRARIES}
)
