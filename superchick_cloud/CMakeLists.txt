cmake_minimum_required(VERSION 2.8.3)
project(superchick_cloud)

OPTION(ENABLE_CXX11 "Enable C++11 support" ON)

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()


find_package(PkgConfig) 
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf
  vicon_bridge
  rospy
  sensor_msgs
  rostime
  rosconsole
  roslib
)


include_directories(${rosconsole_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES superchick_cloud
   CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf vicon_bridge
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(superchick_cloud src/pcd_grabber.cpp
                        # src/superchick_cloud.cpp
                        #         )

# target_link_libraries(superchick_cloud
 #   ${catkin_LIBRARIES} ${PCL_LIBRARIES} 
 #                          ${Boost_LIBRARIES}
 # )
