cmake_minimum_required(VERSION 2.8.3)
project(kinematic_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  cmake_modules
  pluginlib
)

find_package(PkgConfig)
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
add_definitions(${EIGEN_DEFINITIONS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES kinematic_model
 CATKIN_DEPENDS moveit_msgs 
 				roscpp 
 				rospy 
 				moveit_core 
 				moveit_ros_planning
    			moveit_ros_planning_interface
 DEPENDS system_lib Eigen3
)

include_directories(
  SYSTEM 
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ executable
add_executable(kinematic_model src/kinematic_model.cpp)
add_executable(kinematic_api src/kinematic_api.cpp)
add_executable(chick src/chick_planning.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(kinematic_model
  ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES}
)

target_link_libraries(kinematic_api
  ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES}
)
target_link_libraries(chick
  ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES}
)