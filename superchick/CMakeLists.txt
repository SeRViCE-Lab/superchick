cmake_minimum_required(VERSION 2.8.3)
project(superchick)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diff_drive_controller #gazebo_ros
  joint_state_controller
  joint_state_publisher
  position_controllers
  robot_state_publisher
  roslaunch
  rviz
  urdf
  xacro
)

#  CATKIN_DEPENDS controller_manager diff_drive_controllergazebo_ros #joint_state_controller joint_state_publisher position_controllers #robot_state_publisher roslaunch rviz urdf xacro
#  DEPENDS system_lib
#)

include_directories(
  ${catkin_INCLUDE_DIRS}
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