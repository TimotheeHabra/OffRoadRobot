/*
 * Useful functions used to create vectors and tabulars
 * and to release the memory (free) - header
 *
 * author: Nicolas Van der Noot
 */
#include <stdlib.h>
#include <stdio.h>

char*** create_triple_char(int a, int *nb_var_ctrl, int c);
void free_triple_char(char ***triple_tab, int a, int *nb_var_ctrl);

char** create_double_char(int a, int b);
void free_double_char(char **double_tab, int a);

int** create_double_int(int a, int *nb_var_ctrl);
void free_double_int(int **double_int, int a);

int*** create_triple_int(int a, int *nb_var_ctrl, int c);
void free_triple_int(int ***triple_int, int a, int *nb_var_ctrl);