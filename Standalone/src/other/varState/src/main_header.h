/*
 * Header of the main file used to generate 
 *     - c files
 *           'user_sf_IO.c', 'user_sf_IO.h', 'ControllersStruct.c' and 'ControllersStruct.h'
 *     - Matlab files
 *           'simu_variables.m' and 'control_variables.m'
 *
 * author: Nicolas Van der Noot
 */	
#include "useful_functions.h"
#include "read_ctrl_functions.h"
#include "read_simu_functions.h"
#include "print_matlab.h"
#include "print_c_files.h"


void generate_simu_variables_m();
void generate_control_variables_m();

void generate_user_sf_IO();
void generate_ControllersStruct();

void free_ctrl_var(int nb_ctrl, int *nb_var_ctrl, char ***varNameTab, char ***typeTab, char ***strTypeTab,
	char **namesCtrl, int **varSizeTab, int ***twoVarSizeTab);
void free_simu_var(int *nb_var_simu, int **varSizeTab, char ***varNameTab, char ***typeTab);