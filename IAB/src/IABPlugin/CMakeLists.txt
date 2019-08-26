cmake_minimum_required(VERSION 3.1)
project(IABPlugin VERSION 1.0)

# Find and load CMake configuration of packages containing this plugin's dependencies
find_package(SofaGui REQUIRED) # Dependency to SofaGuiQt
find_package(Qt5 COMPONENTS Core) # Dependency to Qt5Core

# List all files
set(IABPlugin_SRC_DIR src/IABPlugin)
set(HEADER_FILES
    ${IABPlugin_SRC_DIR}/config.h
    ${IABPlugin_SRC_DIR}/MyBehaviorModel.h
    ${IABPlugin_SRC_DIR}/MyMappingPendulumInPlane.h
    ${IABPlugin_SRC_DIR}/MyMappingPendulumInPlane.inl
    ${IABPlugin_SRC_DIR}/MyProjectiveConstraintSet.h
    ${IABPlugin_SRC_DIR}/MyProjectiveConstraintSet.inl
    )
set(HEADER_FILES_TO_MOC
    ${IABPlugin_SRC_DIR}/MyDataWidgetUnsigned.h
    )
set(SOURCE_FILES
    ${IABPlugin_SRC_DIR}/initPlugin.cpp
    ${IABPlugin_SRC_DIR}/MyBehaviorModel.cpp
    ${IABPlugin_SRC_DIR}/MyDataWidgetUnsigned.cpp
    ${IABPlugin_SRC_DIR}/MyMappingPendulumInPlane.cpp
    ${IABPlugin_SRC_DIR}/MyProjectiveConstraintSet.cpp
    )
set(README_FILES
    IABPlugin.txt
    )

# [Qt] Create moc code.
qt5_wrap_cpp(MOCCED_HEADER_FILES ${HEADER_FILES_TO_MOC})

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES_TO_MOC} ${MOCCED_HEADER_FILES} ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

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
iab_generate_package(
    NAME ${PROJECT_NAME}
    TARGETS ${PROJECT_NAME}
    VERSION ${PROJECT_VERSION}
    INCLUDE_SOURCE_DIR ${IABPlugin_SRC_DIR}
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )
