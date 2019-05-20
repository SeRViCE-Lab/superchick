cmake_minimum_required(VERSION 3.1)
project(IAB8)

add_definitions(-std=c++14)

find_package(Boost COMPONENTS filesystem system REQUIRED)

if(APPLE)
		set(userdir "/Users/$ENV{USER}")
else()
		set(userdir "/home/$ENV{USER}")
endif()

list(APPEND CMAKE_PREFIX_PATH "${userdir}/sofa/master/build/install/lib/cmake")
list(APPEND CMAKE_MODULE_PATH "${userdir}/sofa/master/build/cmake/Modules")
set(CMAKE_INSTALL_PREFIX "${userdir}/sofa/master/build/install")
set(RUN_SOFA_DIR "${userdir}/sofa/master/src/applications/projects/runSofa")
set(RC_FILES "${RUN_SOFA_DIR}/runSofa.icns")
set(RC_FILES "${RUN_SOFA_DIR}/runSofa.rc")

find_package(SofaBase REQUIRED)
find_package(SofaFramework REQUIRED)
find_package(SofaSimulation REQUIRED)
find_package(SofaGui REQUIRED)
find_package(SofaMiscCollision REQUIRED)
find_package(SofaPython REQUIRED)

include(${RUN_SOFA_DIR}/cmake/GeneratePluginConfig.cmake)
# message(STATUS "RUN_SOFA_DIR: ${RUN_SOFA_DIR}")
# message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")


set(_pluginLocation "${userdir}/sofa/master/build/lib")
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

# message(STATUS "Boost Libraries: ${Boost_LIBRARIES}")
# message(STATUS "Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
# message(STATUS "Boost Includes: ${Boost_INCLUDE_DIRS}")

add_executable(${PROJECT_NAME} src/IAB8.cpp)

target_compile_definitions(${PROJECT_NAME} PUBLIC "CONFIG_PLUGIN_FILENAME=${_configPluginFileName}")
target_compile_definitions(${PROJECT_NAME} PUBLIC "DEFAULT_CONFIG_PLUGIN_FILENAME=${_defaultConfigPluginFileName}")

target_link_libraries(${PROJECT_NAME} SofaComponentAdvanced
																			SofaComponentMisc
																			SofaSimulationGraph
																			SofaGuiMain
																			SofaMiscCollision
																			SofaPython
																			SofaSparseSolver
																			SofaPreconditioner
																			${Boost_FILESYSTEM_LIBRARY})
