cmake_minimum_required(VERSION 3.1)
project(IABPlugin VERSION 1.0)

# Find and load CMake configuration of packages containing this plugin's dependencies
find_package(SofaGui REQUIRED PATHS "${CONFIG_PATHS}") # Dependency to SofaGuiQt
find_package(Qt5 COMPONENTS Core Charts) # Dependency to Qt5Core

include_directories(include)

set(HEADER_FILES
    config.h
    BehaviorModel.h
    MappingPendulumInPlane.h
    MappingPendulumInPlane.inl
    ProjectiveConstraintSet.h
    ProjectiveConstraintSet.inl
    )
set(HEADER_FILES_TO_MOC
    DataWidgetUnsigned.h
    )
set(SOURCE_FILES
    src/initPlugin.cpp
    src/BehaviorModel.cpp
    src/DataWidgetUnsigned.cpp
    src/MappingPendulumInPlane.cpp
    src/ProjectiveConstraintSet.cpp
    )
set(README_FILES
    IABPlugin.txt
    )

# [Qt] Create moc code.
qt5_wrap_cpp(MOCCED_HEADER_FILES ${HEADER_FILES_TO_MOC})

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES_TO_MOC}
																		${MOCCED_HEADER_FILES}
																		${HEADER_FILES}
																		${SOURCE_FILES}
																		${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
# set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_IABPLUGIN")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} SofaGuiQt)

# If using the build of this plugin with "find_package(IABPlugin)",
# then discover includes from '${CMAKE_CURRENT_SOURCE_DIR}/src' directory.
# target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")

# If using the install of this plugin with "find_package(IABPlugin)",
# then discover includes from 'include' directory.
# target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")
