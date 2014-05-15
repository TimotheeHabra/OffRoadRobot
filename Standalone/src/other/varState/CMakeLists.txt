#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# CMakeLists used to generated the executable in charge of automatically
# generating the new versions of 'ControllersStruct.h/.c' and 'user_sf_IO.h/.c'
# when 'control_variables.txt' or 'simu_variables.txt' are modified
#

# CMake minimum version
cmake_minimum_required(VERSION 2.8.7)

# project name
project (GEN_STATE_VAR)

# message to display the project name
message(STATUS "Processing ${PROJECT_NAME}")

# use Release version and not the Debug one (faster)
set(CMAKE_BUILD_TYPE Release)

# for Unix: display all the warnings, except the ones related to /* -- */
if (UNIX)
  set(CMAKE_C_FLAGS "-g -Wall -Wno-comment")
endif (UNIX)

# function to list all source files in the src/ directory, 
# recursing into sub-directories
function(list_gen_source_files HEADER_FILES)
    file(GLOB_RECURSE SOURCE_FILES_TMP "${PROJECT_SOURCE_DIR}/src/*.c" "${PROJECT_SOURCE_DIR}/src/*.cpp")
    set(SOURCE_FILES ${SOURCE_FILES_TMP} PARENT_SCOPE)
endfunction()

# function to list all paths to header files in the src/ directory, 
# recursing into sub-directories (includes)
function(list_gen_include_directories FULL_INCLUDE_DIRECTORIES)
    file(GLOB_RECURSE FULL_INCLUDE_DIRECTORIES_TMP RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/src/*.h" "${PROJECT_SOURCE_DIR}/src/*.hpp")
    set(DIR_LIST "")
    foreach(FILE_PATH ${FULL_INCLUDE_DIRECTORIES_TMP})
        get_filename_component(DIR_PATH ${FILE_PATH} PATH)
        set(DIR_LIST ${DIR_LIST} ${DIR_PATH})
    endforeach(FILE_PATH)
    list(REMOVE_DUPLICATES DIR_LIST) 
    set(FULL_INCLUDE_DIRECTORIES ${DIR_LIST} PARENT_SCOPE)
endfunction()
 
# list source files to compile
list_gen_source_files(SOURCE_FILES)

# list include directories (to find headers)
list_gen_include_directories(FULL_INCLUDE_DIRECTORIES)

# include these directories
include_directories (${FULL_INCLUDE_DIRECTORIES})

# name of the executable: 'Gen_state_var'
set (Executable Gen_state_var)

# generate the executable
add_executable (${Executable} ${SOURCE_FILES})

# math external library (for Unix): linkage
if (UNIX)
	target_link_libraries (${Executable} m)
endif (UNIX)
