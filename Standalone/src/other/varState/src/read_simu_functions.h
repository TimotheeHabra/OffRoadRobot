/*
 * Headers of the functions used to read 'simu_variables.txt'
 *
 * author: Nicolas Van der Noot
 */
#include <string.h>

#define FGETS_MAX_SIZE 1000 // maximal size for a line
#define STRING_MAX_SIZE 100  // maximal size for a word

int* read_nb_var_simu(char *fileName);
void read_simu_variables(char *fileName, int *nb_var_simu, char ***varNameTab, char ***typeTab, int **varSizeTab);
