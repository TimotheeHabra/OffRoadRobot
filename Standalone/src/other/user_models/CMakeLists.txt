#
# author: 
# Feb 27 2014
#
# CMakeLists used to generate the executable in charge of automatically
# generating the new versions of 'ControllersStruct.h/.c' and 'user_sf_IO.h/.c'
# when 'control_variables.txt' or 'simu_variables.txt' are modified
#

# CMake minimum version
cmake_minimum_required(VERSION 2.8.7)

# project name
project (GEN_MDS_USER)

# message to display the project name
message(STATUS "Processing ${PROJECT_NAME}")

# use Release version and not the Debug one (faster)
set(CMAKE_BUILD_TYPE Debug)

# for Unix: display all the warnings, except the ones related to /* -- */
if (UNIX)
  set(CMAKE_C_FLAGS "-g -Wall -Wno-comment")
endif (UNIX)

# define the location of all the 'Find<module>.cmake' files
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../conf)

# Libxml2 (required for .mbs loading)
find_package(Libxml2 REQUIRED)
if (LIBXML2_FOUND)
    include_directories (${LIBXML2_INCLUDE_PATH})   
    message( "\n  >> Libxml2 found")
else (LIBXML2_FOUND)
    message( "\n  >> Libxml2 not found, modify src/other/conf/FindLibxml2.cmake")
endif(LIBXML2_FOUND)

message("      -> INC : ${LIBXML2_INCLUDE_PATH}")
message("      -> LIB : ${LIBXML2_LIBRARIES}\n")

# function to list all source files in the src/ directory, 
# recursing into sub-directories
function(list_gen_source_files HEADER_FILES)
    file(GLOB_RECURSE SOURCE_FILES_TMP 
		"${PROJECT_SOURCE_DIR}/src/*.c" 
		"${PROJECT_SOURCE_DIR}/src/*.cpp"
		"${PROJECT_SOURCE_DIR}/../../generic/mbs_load_xml/mbs_xml_reader.c"
		"${PROJECT_SOURCE_DIR}/../../generic/other/useful_functions.c")
    set(SOURCE_FILES ${SOURCE_FILES_TMP} PARENT_SCOPE)
endfunction()

# function to list all paths to header files in the src/ directory, 
# recursing into sub-directories (includes)
function(list_gen_include_directories FULL_INCLUDE_DIRECTORIES)
    file(GLOB_RECURSE FULL_INCLUDE_DIRECTORIES_TMP RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
		"${PROJECT_SOURCE_DIR}/src/*.h"
		"${PROJECT_SOURCE_DIR}/src/*.hpp"
		"${PROJECT_SOURCE_DIR}/../../generic/mbs_load_xml/mbs_xml_reader.h"
		"${PROJECT_SOURCE_DIR}/../../generic/other/useful_functions.h"
		"${PROJECT_SOURCE_DIR}/../../project/project_files/project_info.h"
		)
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

# name of the executable: 'Gen_mds_user'
set (Executable Gen_mds_user)

# generate the executable
add_executable (${Executable} ${SOURCE_FILES})

target_link_libraries (${Executable} ${LIBXML2_LIBRARIES})

# math external library (for Unix): linkage
if (UNIX)
	target_link_libraries (${Executable} m)
endif (UNIX)
