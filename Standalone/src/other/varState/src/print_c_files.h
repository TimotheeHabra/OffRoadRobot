/*
 * Header of the functions used to generate 'ControllersStruct.c', 'ControllersStruct.h'
 * 'user_sf_IO.c' and 'user_sf_IO.h'
 *
 * author: Nicolas Van der Noot
 */
void print_c_ctrl_variables(int nb_ctrl, int *nb_var_ctrl, char ***varNameTab, char ***typeTab, 
	int **varSizeTab, int ***twoVarSizeTab, char **namesCtrl, char *fileoutC, char *fileoutH);
void print_c_simu_variables(int *nb_var_simu, char ***varNameTab, char ***typeTab, 
	int **varSizeTab, char *fileoutC, char *fileoutH);
