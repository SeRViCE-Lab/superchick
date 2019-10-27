cmake_minimum_required(VERSION 3.1)
if(APPLE)
	set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
elseif(UNIX AND NOT APPLE)
	set(CMAKE_CXX_COMPILER "/usr/bin/g++")
endif()
project(IAB VERSION 1.0)

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

# for boost ode OdeSolver
find_package(Boost COMPONENTS #filesystem
															#system
															#serialization
															thread REQUIRED)

set(PLUGINS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/plugins")

include(${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/macros.cmake)
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include"
										"${PLUGINS_DIR}"
  									"${Boost_INCLUDE_DIRS}")

set(SOURCE_FILES
		"${PLUGINS_DIR}/IABPlugin/src/initIABPlugin.cpp"
		"${PLUGINS_DIR}/IABPlugin/ForceFields/src/IsochoricForceField.cpp"
		"${PLUGINS_DIR}/IABPlugin/behavior/IABConstraint.cpp"
		"${PLUGINS_DIR}/IABPlugin/behavior/IABBaseConstraint.cpp"
		)
# [Qt] Create moc code.
qt5_wrap_cpp(MOCCED_HEADER_FILES ${HEADER_FILES_TO_MOC})

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED  #${HEADER_FILES}
																		${HEADER_FILES_TO_MOC}
																		${MOCCED_HEADER_FILES}
																		${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SofaGuiQt)


set(SOFA_LIBS SofaComponentAdvanced
							SofaComponentMisc
							SofaSimulationGraph
							SofaGuiMain
							)

add_executable("${PROJECT_NAME}_Launcher" src/IAB.cpp)
target_link_libraries("${PROJECT_NAME}_Launcher" ${SOFA_LIBS})

add_executable(SingleIAB src/SingleIAB.cpp)
target_link_libraries(SingleIAB ${SOFA_LIBS})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION "$ENV{SOFA_ROOT}/lib")

install(DIRECTORY "$ENV{SOFA_ROOT}/lib/" DESTINATION "$ENV{SOFA_ROOT}/lib"
				FILES_MATCHING PATTERN "*.so")
