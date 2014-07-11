cmake_minimum_required(VERSION 2.8.7)

set(PROJECTNAME basic)
set(LIB_NAME motorControlHelpers)

project(${PROJECTNAME})

find_package(YARP REQUIRED)

# add local include
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/Helper ${YARP_INCLUDE_DIRS})

add_library(${LIB_NAME} basic.cpp Helper/MotorControlHelper.cpp )
#add_executable(${PROJECTNAME} main.cpp )
#TARGET_LINK_LIBRARIES(${PROJECTNAME} ${LIB_NAME} ${YARP_LIBRARIES} ${ACE_LIBRARIES})
