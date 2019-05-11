cmake_minimum_required(VERSION 3.1)
project(iab_single)

# add_definitions(-std=c++14)

if(APPLE)
	list(APPEND CMAKE_PREFIX_PATH Users/$ENV{USER}/sofa/v18.06/src/build)
else()
	list(APPEND CMAKE_PREFIX_PATH /home/$ENV{USER}/sofa/v18.06/build/cmake)
	set(INSTALL_FOLDER /home/$ENV{USER}/sofa/v18.06/build/install)
endif()

include_directories(
  ${Boost_INCLUDE_DIRS}
	)
include_directories("${INSTALL_FOLDER}/include")
# link_directories("${INSTALL_FOLDER}/../lib")
set(SOFA_LIBS_DIR "${INSTALL_FOLDER}/../lib")

add_executable(${PROJECT_NAME} Main.cpp)
target_link_libraries(${PROJECT_NAME} ${SOFA_LIBS_DIR} ${Boost_LIBRARIES})

#rm * -rf; cmake -DCMAKE_PREFIX_PATH=/home/${USER}/sofa/v18.06/build/cmake ../; make
