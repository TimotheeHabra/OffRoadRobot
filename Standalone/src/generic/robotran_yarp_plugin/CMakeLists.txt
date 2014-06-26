set(PROJECT_NAME robotranYarpPlugin)
project(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.6)
find_package(YARP REQUIRED)


include_directories(${YARP_INCLUDE_DIRS})

set(robotranMotionControlPlugin_source src/RobotranMotionControlBoard.cpp)
set(robotranForceTorquePlugin_source src/ForceTorqueDriver.cpp)

set(robotranMotionControlPlugin_header include/RobotranMotionControlBoard.h)
set(robotranForceTorquePlugin_header include/ForceTorqueDriver.h)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${YARP_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(robotranMotionControlPlugin ${robotranMotionControlPlugin_source} ${robotranMotionControlPlugin_header} )
add_library(robotranForceTorquePlugin ${robotranForceTorquePlugin_source} ${robotranForceTorquePlugin_header} )