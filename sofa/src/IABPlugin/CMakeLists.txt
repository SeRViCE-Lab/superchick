cmake_minimum_required(VERSION 3.1)
project(IABPlugin VERSION 1.0)

# Find and load CMake configuration of packages containing this plugin's dependencies
find_package(SofaGui REQUIRED PATHS "${CONFIG_PATHS}") # Dependency to SofaGuiQt
find_package(Qt5 COMPONENTS Core Charts) # Dependency to Qt5Core

# List all files
set(IABPlugin_SRC_DIR src)
set(HEADER_FILES
    ${IABPlugin_SRC_DIR}/config.h
    ${IABPlugin_SRC_DIR}/BehaviorModel.h
    ${IABPlugin_SRC_DIR}/MappingPendulumInPlane.h
    ${IABPlugin_SRC_DIR}/MappingPendulumInPlane.inl
    ${IABPlugin_SRC_DIR}/ProjectiveConstraintSet.h
    ${IABPlugin_SRC_DIR}/ProjectiveConstraintSet.inl
    )
set(HEADER_FILES_TO_MOC
    ${IABPlugin_SRC_DIR}/DataWidgetUnsigned.h
    )
set(SOURCE_FILES
    ${IABPlugin_SRC_DIR}/initPlugin.cpp
    ${IABPlugin_SRC_DIR}/BehaviorModel.cpp
    ${IABPlugin_SRC_DIR}/DataWidgetUnsigned.cpp
    ${IABPlugin_SRC_DIR}/MappingPendulumInPlane.cpp
    ${IABPlugin_SRC_DIR}/ProjectiveConstraintSet.cpp
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
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_IABPLUGIN")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} SofaGuiQt)

# If using the build of this plugin with "find_package(IABPlugin)",
# then discover includes from '${CMAKE_CURRENT_SOURCE_DIR}/src' directory.
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")

# If using the install of this plugin with "find_package(IABPlugin)",
# then discover includes from 'include' directory.
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

# Create package Config, Version & Target files.
# Deploy the headers, resources, scenes & examples.
# Set the plugin 'relocatable' if built within SOFA.
# --> see SofaMacros.cmake
# iab_generate_package(
#     NAME ${PROJECT_NAME}
#     TARGETS ${PROJECT_NAME}
#     VERSION ${PROJECT_VERSION}
#     INCLUDE_SOURCE_DIR ${IABPlugin_SRC_DIR}
#     INCLUDE_INSTALL_DIR ${PROJECT_NAME}
#     RELOCATABLE "plugins"
#     )
# iab_install_targets("${PROJECT_NAME}" "${ARG_TARGETS}" "${include_install_dir}" "${ARG_INCLUDE_SOURCE_DIR}" "${ARG_IAB_INSTALL_DIR}" "${ARG_RELOCATABLE}")
# iab_write_package_config_files("${ARG_NAME}" "${ARG_VERSION}")
