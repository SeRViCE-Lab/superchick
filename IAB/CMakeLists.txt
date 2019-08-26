cmake_minimum_required(VERSION 3.1)
project(IAB)

add_definitions(-std=c++14)

find_package(Boost COMPONENTS filesystem system REQUIRED)

 # best to read SOFA_ROOTfrom cmd line due to different requirements
 if("${SOFA_ROOT}" STREQUAL "")
 	# see if it can be retrieved from ENV VAR
	if($ENV{SOFA_ROOT} STREQUAL "")
	 	message(FATAL_ERROR "\nYou must explicitly provide the path to sofa root\n"
		 					"e.g. cmake -DSOFA_ROOT=/sofa /path/to/CMakeLists.txt")
	else()
		set(SOFA_ROOT "$ENV{SOFA_ROOT}")
	endif()
endif()


if(APPLE)
	list(APPEND CMAKE_MODULE_PATH "${SOFA_ROOT}/cmake")
	list(APPEND CMAKE_PREFIX_PATH "${SOFA_ROOT}/cmake")
	set(RC_FILES "${RUN_SOFA_ROOT}/runSofa.icns")
	set(RUN_SOFA_ROOT "${SOFA_ROOT}/../applications/projects/runSofa")
elseif(UNIX AND NOT APPLE)
	list(APPEND CMAKE_MODULE_PATH "${SOFA_ROOT}/build/cmake")
	list(APPEND CMAKE_PREFIX_PATH "${SOFA_ROOT}/build/install/lib/cmake")
	set(RC_FILES "${RUN_SOFA_ROOT}/runSofa.rc")
	set(RUN_SOFA_ROOT "${SOFA_ROOT}/applications/projects/runSofa")
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

find_package(SofaBase REQUIRED)
find_package(SofaFramework REQUIRED)
find_package(SofaSimulation REQUIRED)
find_package(SofaGui REQUIRED)
find_package(SofaMiscCollision REQUIRED)
find_package(SofaPython REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS  Charts)

# find_package(SofaComponentAdvanced)
# find_package(SofaComponentMisc)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/macros.cmake)

macro(iab_add_plugin directory plugin_name)
    iab_add_generic( ${directory} ${plugin_name} "Plugin" ${ARGV2} )
endmacro()

# add iab plugin
iab_add_plugin(src/IABPlugin IAB)

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
							# SofaSparseSolver
							# SofaPreconditioner
							)

add_executable(${PROJECT_NAME} src/IAB.cpp)
# target_link_libraries(${PROJECT_NAME} SofaComponentAdvanced SofaComponentMisc)
# target_link_libraries(${PROJECT_NAME} SofaSimulationGraph)
# target_link_libraries(${PROJECT_NAME} SofaGuiMain)
target_link_libraries(${PROJECT_NAME} ${SOFA_LIBS}											${Boost_FILESYSTEM_LIBRARY})
