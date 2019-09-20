cmake_minimum_required(VERSION 3.1)
project(IsochoricForceField VERSION 1.0)

# include_directories(include)
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
add_compile_options(-std=c++14)
# Find and load CMake configuration of packages containing this plugin's dependencies
find_package(SofaGui REQUIRED PATHS "${CONFIG_PATHS}") # Dependency to SofaGuiQt
find_package(Qt5 COMPONENTS Core Charts) # Dependency to Qt5Core

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(HEADER_FILES
    include/config.h
    include/IsochoricForceField.h
    include/IsochoricForceField.inl
    )

set(SOURCE_FILES
    src/IsochoricForceField.cpp
    )

set(SOFA_LIBS SofaComponentAdvanced
							SofaComponentMisc
							SofaSimulationGraph
							SofaGuiMain
							# SofaMiscCollision
							SofaGuiQt
							)

# Create the plugin library.
add_library(${PROJECT_NAME} "SHARED" ${SOURCE_FILES})
# set(INTEGRATOR_LIBS "${CMAKE_CURRENT_BUILD_DIR}/lib")
# message(STATUS "Ext libs: ${INTEGRATOR_LIBS}")

target_link_libraries(${PROJECT_NAME} ${SOFA_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

# target_sources(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_BUILD_DIR}/lib")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_IABFORCEFIELD")

# install(TARGETS ${PROJECT_NAME}
#         RUNTIME DESTINATION bin
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib)
# install(DIRECTORY "lib/" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}"
# 				FILES_MATCHING PATTERN "*.so;dylib")
install(DIRECTORY "lib/" DESTINATION "$ENV{SOFA_ROOT}/lib"
				FILES_MATCHING PATTERN "*.so")
