cmake_minimum_required(VERSION 2.8.3)
project(lab3)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs 
	genmsg 
	tf)

generate_messages(DEPENDENCIES std_msgs)

catkin_package( )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(nonkdl src/nonkdl.cpp)
target_link_libraries(nonkdl ${catkin_LIBRARIES})
add_dependencies(nonkdl lab3_generate_messages_cpp)

