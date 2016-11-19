cmake_minimum_required(VERSION 2.8.3)
project(superchick)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  joint_state_controller
  joint_state_publisher
  position_controllers
  robot_state_publisher
  roslaunch
  rviz
  urdf
  xacro
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)



include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
    DEPENDS
      gazebo_ros
    CATKIN_DEPENDS
      joint_state_controller 
      joint_state_publisher 
      position_controllers 
      robot_state_publisher 
      roslaunch 
      rviz 
      urdf 
      xacro
    INCLUDE_DIRS
    LIBRARIES
)

# Xacro files # this is a hack here so world files are not dumped to \
#src dir
file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/*world.xacro)

foreach(it ${xacro_files})
  # remove .xacro extension
  string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
  set(output_filename ${CMAKE_MATCH_1})

  # create a rule to generate ${output_filename} from {it}
  xacro_add_xacro_file(${it} ${output_filename})

  list(APPEND world_files ${output_filename})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(fathom_files ALL DEPENDS ${world_files})