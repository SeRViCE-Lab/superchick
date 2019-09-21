cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
project(IAB 	VERSION 1.0)

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

set(CMAKE_INSTALL_PREFIX "${SOFA_ROOT}/install;${SOFA_ROOT}/cmake")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

find_package(SofaFramework REQUIRED PATHS "${CONFIG_PATHS}")
find_package(SofaBase REQUIRED PATHS "${CONFIG_PATHS}")
find_package(SofaSimulation REQUIRED PATHS "${CONFIG_PATHS}")
find_package(SofaGui REQUIRED PATHS "${CONFIG_PATHS}")
find_package(SofaMiscCollision REQUIRED PATHS "${CONFIG_PATHS}")
find_package(SofaPython REQUIRED PATHS "${CONFIG_PATHS}") # why are we using this?
find_package(Qt5 REQUIRED COMPONENTS Core Charts)

set(PLUGINS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/plugins")

include(${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/macros.cmake)
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES(FORCEFIELDS_INCLUDE "${PLUGINS_DIR}/ForceFields")
INCLUDE_DIRECTORIES(FORCEFIELDS_INCLUDE "${PLUGINS_DIR}/IABPlugin")

set(IABPLUGIN_INCLUDES  "${PLUGINS_DIR}/IABPlugin/include/initIABPlugin.h"
												"${PLUGINS_DIR}/ForceFields/include/config.h"
												"${PLUGINS_DIR}/ForceFields/include/integrand.inl"
												"${PLUGINS_DIR}/ForceFields/include/IsochoricForceField.h"
												"${PLUGINS_DIR}/ForceFields/include/IsochoricForceField.inl")

set(SOURCE_FILES
		${PLUGINS_DIR}/IABPlugin/src/InitIABPlugin.cpp
		${PLUGINS_DIR}/ForceFields/src/IsochoricForceField.cpp
		)

INCLUDE_DIRECTORIES("${IABPLUGIN_INCLUDES}")

# [Qt] Create moc code.
qt5_wrap_cpp(MOCCED_HEADER_FILES ${HEADER_FILES_TO_MOC})

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES_TO_MOC}
																		${MOCCED_HEADER_FILES}
																		${IABPLUGIN_INCLUDES}
																		${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SofaGuiQt)


set(SOFA_LIBS SofaComponentAdvanced
							SofaComponentMisc
							SofaSimulationGraph
							SofaGuiMain
							)

add_executable("${PROJECT_NAME}_Launcher" src/IAB.cpp)
target_link_libraries("${PROJECT_NAME}_Launcher" ${SOFA_LIBS})
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION "$ENV{SOFA_ROOT}/lib")

install(DIRECTORY "$ENV{SOFA_ROOT}/lib/" DESTINATION "$ENV{SOFA_ROOT}/lib"
				FILES_MATCHING PATTERN "*.so;*.dylib")
