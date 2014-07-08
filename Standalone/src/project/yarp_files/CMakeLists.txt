set(PROJECT_NAME robotranYarpPlugin)
project(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.6)
find_package(YARP REQUIRED)

include(YarpInstallationHelpers)
yarp_configure_external_installation(robotran)

message(INFO " ROBOTRAN_ROBOTS_INSTALL_DIR is ${ROBOTRAN_ROBOTS_INSTALL_DIR} ||")
message(INFO " appname is ${appname} ||")
yarp_install(DIRECTORY conf/ DESTINATION ${ROBOTRAN_ROBOTS_INSTALL_DIR}/${appname} PATTERN .ini)




#include_directories(${YARP_INCLUDE_DIRS})
#set(yarp_file_sourcedir  ${CMAKE_SOURCE_DIR}/src/project/yarp_files/)
#set(yarp_helper_source ${yarp_file_sourcedir}/yarp_init.cpp ${yarp_file_sourcedir}/yarp_finish.cpp ${yarp_file_sourcedir}/yarp_update.cpp)
#set(robotranMotionControlPlugin_source src/RobotranMotionControlBoard.cpp)
#set(robotranForceTorquePlugin_source src/ForceTorqueDriver.cpp)

#set(robotranMotionControlPlugin_header include/RobotranMotionControlBoard.h)
#set(robotranForceTorquePlugin_header include/ForceTorqueDriver.h)

#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${YARP_DIR})

#include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#add_library(yarpHelper  ${yarp_helper_source} )
#add_library(robotranMotionControlPlugin ${robotranMotionControlPlugin_source} ${robotranMotionControlPlugin_header} )
#add_library(robotranForceTorquePlugin ${robotranForceTorquePlugin_source} ${robotranForceTorquePlugin_header} )

#target_link_libraries(robotranMotionControlPlugin yarpHelper)
#target_link_libraries(robotranForceTorquePlugin yarpHelper)
