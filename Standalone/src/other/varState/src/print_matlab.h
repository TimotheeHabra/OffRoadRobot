/*
 * Header of the fonctions used to generate the files in Matlab
 * which are the equivalent of 'control_variables.txt' and 'simu_variables.txt'
 *
 * author: Nicolas Van der Noot
 */
void print_matlab_control_variables(int nb_ctrl, int *nb_var_ctrl, char ***varNameTab, char ***typeTab, 
	int **varSizeTab, int ***twoVarSizeTab, char **namesCtrl, char *fileout);
void print_matlab_simu_variables(int *nb_var_simu, char ***varNameTab, char ***typeTab, 
	int **varSizeTab, char *fileout);
