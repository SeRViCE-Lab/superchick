cmake_minimum_required(VERSION 3.1)
project(IAB)

add_compile_options(-std=c++14)

 # best to read SOFA_ROOTfrom cmd line due to different requirements
 if("${SOFA_ROOT}" STREQUAL "")
 	# see if it can be retrieved from ENV VAR
	if($ENV{SOFA_ROOT} STREQUAL "")
	 	message(FATAL_ERROR "\nYou must explicitly provide the path to sofa root build directory if"
		 					"you have not set the SOFA_ROOT envionment name"
		 					"e.g. cmake -DSOFA_ROOT=/sofa /path/to/CMakeLists.txt")
	else()
		set(SOFA_ROOT "$ENV{SOFA_ROOT}")
		message(STATUS "Setting path ${SOFA_ROOT} as sofa root directory")
	endif()
endif()

set(CONFIG_PATHS "${SOFA_ROOT}/cmake")  # prevent conflict with system libraries
list(APPEND CMAKE_MODULE_PATH "${SOFA_ROOT}/cmake")
list(APPEND CMAKE_PREFIX_PATH "${SOFA_ROOT}/install/lib/cmake")
set(RUN_SOFA_ROOT "${SOFA_ROOT}/../applications/projects/runSofa")

if(APPLE)
	set(RC_FILES "${RUN_SOFA_ROOT}/runSofa.icns")
elseif(UNIX AND NOT APPLE)
	set(RC_FILES "${RUN_SOFA_ROOT}/runSofa.rc")
else()
    message(STATUS "Windows not supported -- yet.")
endif()

include(${RUN_SOFA_ROOT}/cmake/GeneratePluginConfig.cmake)

# add definitions for sofa_dir for fwd pickup bu iab.cpp
add_definitions(-DSOFA_ROOT=${SOFA_ROOT})
add_definitions(-DIAB_ROOT=${CMAKE_SOURCE_DIR})

message(STATUS "SOFA_ROOT: ${SOFA_ROOT}")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")


set(CMAKE_INSTALL_PREFIX "${SOFA_ROOT}/build/install;${SOFA_ROOT}/cmake")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

find_package(SofaFramework REQUIRED PATHS "${CONFIG_PATHS}")
find_package(SofaBase REQUIRED PATHS "${CONFIG_PATHS}")
find_package(SofaSimulation REQUIRED PATHS "${CONFIG_PATHS}")
find_package(SofaGui REQUIRED PATHS "${CONFIG_PATHS}")
find_package(SofaMiscCollision REQUIRED PATHS "${CONFIG_PATHS}")
find_package(SofaPython REQUIRED PATHS "${CONFIG_PATHS}") # why are we using this?
find_package(Qt5 REQUIRED COMPONENTS Core Charts)

include(${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/macros.cmake)
INCLUDE_DIRECTORIES("${CMAKE_INSTALL_PREFIX}/include")
INCLUDE_DIRECTORIES(IABPLUGIN_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/plugins")

set(IABPLUGIN_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/plugins/IABPlugin/include")
set(ForceFields_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/plugins/ForceFields")

INCLUDE_DIRECTORIES("${IABPLUGIN_INCLUDE}")
 # work on this post-icra/ra-l deadline
# iab_add_plugin(plugins/IABPlugin IAB)
# add_subdirectory(plugins/IABPlugin)
add_subdirectory(plugins/IABPlugin ${CMAKE_CURRENT_BINARY_DIR}/build/libs)

set(SOFA_LIBS_DIR "${CMAKE_INSTALL_PREFIX}/lib")
link_directories(${SOFA_LIBS_DIR} "${CMAKE_BINARY_DIR}/lib")

add_library(integral "SHARED" src/integrand.cxx) # my local integrator

set(SOFA_LIBS SofaComponentAdvanced
							SofaComponentMisc
							SofaSimulationGraph
							SofaGuiMain
							SofaMiscCollision
							SofaPython
							)

add_executable(${PROJECT_NAME} src/IAB.cpp)
target_link_libraries(${PROJECT_NAME} ${SOFA_LIBS} IABPlugin)

install(DIRECTORY "lib/" DESTINATION "$ENV{SOFA_ROOT}/lib"
				FILES_MATCHING PATTERN "*.so")
#
