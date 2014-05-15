/*
 * Main header of some useful functions
 *
 * author: Nicolas Van der Noot
 */
#ifndef __USEFUL_FUNCTIONS_H_INCLUDED__  // guard against multiple/recursive includes
#define __USEFUL_FUNCTIONS_H_INCLUDED__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// exponents related functions
double pow_int(double a, int b);
int expo_ten_inf(double nb);

// int related functions
int* get_int_vec(int x);
int** get_int_tab(int x, int y);
void free_int_vec(int *vec);
void free_int_tab(int **tab, int x);

// double related functions
double* get_double_vec(int x);
double** get_double_tab(int x, int y);
void free_double_vec(double *vec);
void free_double_tab(double **tab, int x);

// char related functions
char* get_char_vec(int x);
char** get_char_tab(int x, int y);
void free_char_vec(char *vec);
void free_char_tab(char** tab, int x);

#endif
