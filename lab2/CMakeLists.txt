cmake_minimum_required(VERSION 2.8.3)
project(lab2)

find_package(catkin REQUIRED COMPONENTS
  orocos_kdl
  roscpp
)

catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS})
add_executable(dh2rpy src/dh2rpy.cpp)
target_link_libraries(dh2rpy ${catkin_LIBRARIES})
