/*
 * Header of the main file used to generate 
 *     - c files
 *           'user_sf_IO.c', 'user_sf_IO.h', 'ControllersStruct.c' and 'ControllersStruct.h'
 *     - Matlab files
 *           'simu_variables.m' and 'control_variables.m'
 *
 * author: 
 */	


#include "print_c_files.h"

#include <sys/stat.h>

#include "cmake_config.h"
#include "project_info.h"
#define MBS_FILE PROJECT_ABS_PATH"/../dataR/"PROJECT_NAME_MBS

char* get_time_machine();
void print_empty_file(char *fileout);


