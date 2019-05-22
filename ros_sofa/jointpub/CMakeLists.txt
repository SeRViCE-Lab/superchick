cmake_minimum_required(VERSION 2.8.3)
project(jointpub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp tf rospy sensor_msgs std_msgs
)

## Declare a C++ executable
add_executable(jointpub_node src/jointpub.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(jointpub_node
  ${catkin_LIBRARIES}
)
