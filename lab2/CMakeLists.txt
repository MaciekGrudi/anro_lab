cmake_minimum_required(VERSION 2.8.3)
project(lab2)

find_package(catkin REQUIRED 
  COMPONENTS roscpp urdf rosconsole cmake_modules
)

find_package(orocos_kdl REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${orocos_kdl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

add_compile_options(-std=c++11)

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${KDL_LIBRARY}
  CATKIN_DEPENDS roscpp rosconsole urdf
  DEPENDS orocos_kdl
)

add_executable(dh2rpy src/dh2rpy.cpp)
target_link_libraries(dh2rpy ${PROJECT_NAME})

