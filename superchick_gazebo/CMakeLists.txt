cmake_minimum_required(VERSION 2.8.3)
project(superchick_gazebo)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	gazebo_ros
	)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS system_lib
  	roscpp
  	gazebo_ros
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/superchick_gazebo.cpp
 )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)