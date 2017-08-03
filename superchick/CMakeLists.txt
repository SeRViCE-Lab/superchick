cmake_minimum_required(VERSION 2.8.3)
project(superchick)

find_package(catkin REQUIRED COMPONENTS
  # controller_manager
  # gazebo_ros
  # joint_state_controller
  # joint_state_publisher
  # position_controllers
  # robot_state_publisher
  roslaunch
  rviz
  urdf
  xacro
  roslib
)

# Depend on system install of Gazebo
# find_package(gazebo REQUIRED)

# include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})
# add_definitions("-DmjUSEDOUBLE")

set(MUJOCO_SOURCE_DIR $ENV{HOME}/mujoco/mjpro150)

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/)
find_package( Boost COMPONENTS filesystem system serialization thread REQUIRED)
find_package( osg )
find_package( osgViewer )
find_package( OpenThreads )
find_package(osgGA)

set(OSG_LIBRARIES ${OSG_LIBRARY} ${OSGVIEWER_LIBRARY} ${OPENTHREADS_LIBRARY} ${OSGGA_LIBRARY} )
include_directories(SYSTEM ${OSG_INCLUDE_DIR})
include_directories("${MUJOCO_SOURCE_DIR}/include"  ${Boost_INCLUDE_DIRS})
# message("osg includes: ${OSG_INCLUDE_DIR}")
# message("mujoco dir: " ${MUJOCO_SOURCE_DIR})

catkin_package(
    DEPENDS
      # gazebo_ros
    CATKIN_DEPENDS
      # joint_state_controller
      # joint_state_publisher
      # position_controllers
      # robot_state_publisher
      roslaunch
      rviz
      urdf
      xacro
      LIBRARIES mj_viewer
)

add_library(mj_viewer src/mujoco_viewer.cpp)
# add_dependencies(mj_viewer ${catkin_EXPORTED_TARGETS} mj_viewer_gencpp)
target_link_libraries(mj_viewer ${MUJOCO_SOURCE_DIR}/bin/libmujoco150.so ${OSG_LIBRARIES})

add_executable(chick_viewer src/test_chick.cpp)


install(TARGETS mj_viewer
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

target_link_libraries(chick_viewer ${Boost_FILESYSTEM_LIBRARY}  
                      ${MUJOCO_SOURCE_DIR}/bin/libmujoco150.so 
                      ${MUJOCO_SOURCE_DIR}/bin/libglew.so 
                      /home/$ENV{USER}/catkin_ws/devel/lib/libmj_viewer.so
                      ${catkin_LIBRARIES} ${OSG_LIBRARIES} 
                      # -lm -ldl
                      )