/*
 * Headers of the functions used to read 'control_variables.txt'
 *
 * author: Nicolas Van der Noot
 */
#include <string.h>

#define FGETS_MAX_SIZE 1000 // maximal size for a line
#define STRING_MAX_SIZE 100  // maximal size for a word

int read_nb_ctrl(char *fileName);
int* read_nb_var_ctrl(char *fileName, int nb_ctrl);
void read_ctrl_variables(char *fileName, int nb_ctrl, int *nb_var_ctrl, char ***varNameTab, char ***typeTab, int **varSizeTab, int ***twoVarSizeTab, char **namesCtrl);