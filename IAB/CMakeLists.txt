cmake_minimum_required(VERSION 3.1)
project(IAB8)

add_definitions(-std=c++14)

find_package(Boost COMPONENTS filesystem system REQUIRED)

if(APPLE)
	set(SOFA_DIR "$ENV{HOME}/$ENV{USER}/sofa/v19.06")
	list(APPEND CMAKE_MODULE_PATH "${SOFA_DIR}/build/cmake/Modules")
elseif(UNIX AND NOT APPLE)
	# check if we are on docker or a host
	execute_process(COMMAND ${CMAKE_COMMAND} -E echo uname -n OUTPUT_VARIABLE NODE_TYPE)
	if(${NODE_TYPE} $ENV{USER})
	set(SOFA_DIR "$ENV{HOME}/$ENV{USER}/sofa/v19.06")
	list(APPEND CMAKE_MODULE_PATH "${SOFA_DIR}/build/cmake")
else()
    message(STATUS "Windows not supported")
endif()

# add definitions for sofa_dir for fwd pickup bu iab.cpp
add_definitions(-DSOFADIR=${SOFA_DIR})

message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")


list(APPEND CMAKE_PREFIX_PATH "${SOFA_DIR}/build/install/lib/cmake")
set(CMAKE_INSTALL_PREFIX "${SOFA_DIR}/build/install")
set(RUN_SOFA_DIR "${SOFA_DIR}/applications/projects/runSofa")
set(RC_FILES "${RUN_SOFA_DIR}/runSofa.icns")
set(RC_FILES "${RUN_SOFA_DIR}/runSofa.rc")

find_package(SofaBase REQUIRED)
find_package(SofaFramework REQUIRED)
find_package(SofaSimulation REQUIRED)
find_package(SofaGui REQUIRED)
find_package(SofaMiscCollision REQUIRED)
find_package(SofaPython REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS  Charts)

include(${RUN_SOFA_DIR}/cmake/GeneratePluginConfig.cmake)
# message(STATUS "RUN_SOFA_DIR: ${RUN_SOFA_DIR}")
# message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")


set(_pluginLocation "${SOFA_DIR}/build/lib")
set(_configPluginFileName "${_pluginLocation}/plugin_list.conf")
set(_defaultConfigPluginFileName "${_configPluginFileName}.default")
set(_defaultConfigPluginFilePath "${_defaultConfigPluginFileName}")
sofa_generate_plugin_config(${_defaultConfigPluginFilePath})
message("Write Plugin list at ${_defaultConfigPluginFilePath}")


if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

include_directories("${CMAKE_INSTALL_PREFIX}/include")
set(SOFA_LIBS_DIR "${CMAKE_INSTALL_PREFIX}/lib")
link_directories(${SOFA_LIBS_DIR})

set(SOFA_LIBS SofaComponentAdvanced
							SofaComponentMisc
							SofaSimulationGraph
							SofaGuiMain
							SofaMiscCollision
							SofaPython
							SofaSparseSolver
							SofaPreconditioner)

add_executable(${PROJECT_NAME} src/IAB.cpp)

target_compile_definitions(${PROJECT_NAME} PUBLIC "CONFIG_PLUGIN_FILENAME=${_configPluginFileName}")
target_compile_definitions(${PROJECT_NAME} PUBLIC "DEFAULT_CONFIG_PLUGIN_FILENAME=${_defaultConfigPluginFileName}")
target_link_libraries(${PROJECT_NAME} ${SOFA_LIBS} ${Boost_FILESYSTEM_LIBRARY})
