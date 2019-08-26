cmake_minimum_required(VERSION 3.1)

#replace here with your project name and version number
project(MyPlugin VERSION X.X)

#look for all packages on which your project depends
find_package(SofaFramework REQUIRED)

set(HEADER_FILES
    initFEDPlugin.h
    #MyHeaderFile2.h
    #MyHeaderFile3.h
)

set(SOURCE_FILES
    initFEDPlugin.cpp
    #MySourceFile2.cpp
    #MySourceFile3.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SofaCore)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_MYPLUGIN")
