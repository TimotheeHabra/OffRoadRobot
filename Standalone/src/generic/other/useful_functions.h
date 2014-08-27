/*
 * Main header of some useful functions
 *
 * author: Nicolas Van der Noot
 */
#ifndef __USEFUL_FUNCTIONS_H_INCLUDED__  // guard against multiple/recursive includes
#define __USEFUL_FUNCTIONS_H_INCLUDED__

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))



#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include <limits.h>

#define DBL_MAX         1.7976931348623158e+308

// exponents related functions
double pow_int(double a, int b);
int expo_ten_inf(double nb);

// int related functions
int* get_int_vec(int x);
int** get_int_tab(int x, int y);
void free_int_vec(int *vec);
void free_int_tab(int **tab, int x);
void copy_int_vec(int *vec1, int *vec2, int x);
void copy_int_tab(int **tab1, int **tab2, int x, int y);
void print_int_vec(int *vec, int x);
void print_int_tab(int **tab, int x, int y);
void sort_int_vec(int *vec1, int *vec2, int x);
void f0123_int_vec(int* vec, int x);
void conc_int_vec(int* vec1, int x1, int* vec2, int x2, int* vec3);
void slct_int_vec(int* vec1, int x1, int* vec2, int x2, int* vec3);
int find_int_vec(int* vec, int x, int f);


// double related functions
double* get_double_vec(int x);
double** get_double_tab(int x, int y);
void free_double_vec(double *vec);
void free_double_tab(double **tab, int x);
void copy_double_vec(double *vec1, double *vec2, int x);
void copy_double_tab(double **tab1, double **tab2, int x, int y);
void print_double_vec(double *vec, int x);
void save_double_vec(double *vec, int x,char *name);
void print_double_tab(double **tab, int x, int y);
void slct_double_vec(double* vec1, int x1, int* vec2, int x2, double* vec3);
void slctc_double_tab(double **tab1, int x1, int y1, double **tab2, int y2, int* vec);
void slctr_double_tab(double **tab1, int x1, int y1, double **tab2, int x2, int* vec);
void zeros_double_tab(double **tab, int x, int y);
void save_double_tab(double **tab, int x, int y, char *name);
int any_double_vec(double *vec, int x);
void zeros_double_vec(double *vec, int x);
double norm_double_vec(double *vec, int x);

double sign(double a);

// char related functions
char* get_char_vec(int x);
char** get_char_tab(int x, int y);
void free_char_vec(char *vec);
void free_char_tab(char** tab, int x);

#endif
