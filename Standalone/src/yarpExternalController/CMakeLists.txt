cmake_minimum_required(VERSION 2.8.7)

set(PROJECTNAME externalControl)
project(${PROJECTNAME})

find_package(YARP REQUIRED)

# add local include
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/Helper ${YARP_INCLUDE_DIRS})


add_executable(${PROJECTNAME} main.cpp externalControl.cpp Helper/InterfacesHelper.cpp)
TARGET_LINK_LIBRARIES(${PROJECTNAME}  ${YARP_LIBRARIES} ${ACE_LIBRARIES})
