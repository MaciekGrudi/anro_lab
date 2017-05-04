cmake_minimum_required(VERSION 2.8.3)
project(lab3)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs 
	genmsg 
	tf
	kdl_parser)

generate_messages(DEPENDENCIES std_msgs)

catkin_package( )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(nonkdl src/nonkdl.cpp)
add_executable(kdl src/kdl.cpp)
target_link_libraries(nonkdl ${catkin_LIBRARIES})
target_link_libraries(kdl ${catkin_LIBRARIES})
add_dependencies(nonkdl lab3_generate_messages_cpp)
add_dependencies(kdl lab3_generate_messages_cpp)
