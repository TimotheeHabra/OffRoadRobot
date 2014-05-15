/*
 * Useful functions used to create vectors and tabulars
 * and to release the memory (free) -> functions
 *
 * author: Nicolas Van der Noot
 */
#include "useful_functions.h"

/*
 * create a triple tabular of char
 */ 
char*** create_triple_char(int a, int *nb_var_ctrl, int c)
{
	int i, j;
	char ***triple_tab;

	triple_tab = (char***) malloc(a*sizeof(char**));

	for(i=0; i<a; i++)
	{
		triple_tab[i] = (char**) malloc(nb_var_ctrl[i]*sizeof(char*));

		for(j=0; j<nb_var_ctrl[i]; j++)
		{
			triple_tab[i][j] = (char*) malloc(c*sizeof(char));
		}
	}

	return triple_tab;
}

/*
 * free a triple tabular of char
 */ 
void free_triple_char(char ***triple_tab, int a, int *nb_var_ctrl)
{
	int i, j;

	for(i=0; i<a; i++)
	{
		for(j=0; j<nb_var_ctrl[i]; j++)
		{
			free(triple_tab[i][j]);
		}
		free(triple_tab[i]);
	}
	free(triple_tab);
}

/*
 * create a double tabular of char
 */
char** create_double_char(int a, int b)
{
	int i;
	char **double_char;

	double_char = (char**) malloc(a*sizeof(char*));

	for(i=0; i<a; i++)
	{
		double_char[i] = (char*) malloc(b*sizeof(char));
	}

	return double_char;
}

/*
 * free a double tabular of char
 */
void free_double_char(char **double_tab, int a)
{
	int i;

	for(i=0; i<a; i++)
	{
		free(double_tab[i]);
	}
	free(double_tab);
}

/*
 * create a double tabular of int
 */
int** create_double_int(int a, int *nb_var_ctrl)
{
	int i;

	int **double_int;

	double_int = (int**) malloc(a*sizeof(int*));

	for(i=0; i<a; i++)
	{
		double_int[i] = (int*) malloc(nb_var_ctrl[i]*sizeof(int));
	}

	return double_int;
}

/*
 * free a double tabular of int
 */
void free_double_int(int **double_int, int a)
{
	int i;

	for(i=0; i<a; i++)
	{
		free(double_int[i]);
	}

	free(double_int);
}

/*
 * create a triple tabular of int
 */
int*** create_triple_int(int a, int *nb_var_ctrl, int c)
{
	int i, j;

	int ***triple_int;

	triple_int = (int***) malloc(a*sizeof(int**));

	for(i=0; i<a; i++)
	{
		triple_int[i] = (int**) malloc(nb_var_ctrl[i]*sizeof(int*));

		for(j=0; j<nb_var_ctrl[i]; j++)
		{
			triple_int[i][j] = (int*) malloc(c*sizeof(int));
		}
	}
	return triple_int;
}

/*
 * free a triple tabular of int
 */
void free_triple_int(int ***triple_int, int a, int *nb_var_ctrl)
{
	int i, j;

	for(i=0; i<a; i++)
	{
		for(j=0; j<nb_var_ctrl[i]; j++)
		{
			free(triple_int[i][j]);
		}

		free(triple_int[i]);
	}
	free(triple_int);
}
