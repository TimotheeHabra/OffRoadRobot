/*
 * Useful functions for the simulation
 *
 * author: Nicolas Van der Noot
 */
 #include "useful_functions.h"

/*
 * Returns a^b (same result as pow(a,b), except that b is a integer -> more efficient)
 */
double pow_int(double a, int b)
{
	int i;
	double cur_val;

	// special case
	if (b == 0)
	{
		return 1.0;
	}

	cur_val = a;

	// loop
	for(i=1; i<abs(b); i++)
	{
		cur_val *= a;
	}

	if (b < 0)
	{
		return 1.0 / cur_val;
	}
	else
	{
		return cur_val;
	}
}

/*
 * Returns the exponent (base 10) -> log_10
 * -> this is the inferior exponent 
 */
int expo_ten_inf(double nb)
{
	int iter;
	double cur_nb;

	nb = fabs(nb);
	
	iter = 0;

	if (nb < 1.0)
	{
		cur_nb = 1.0;

		// loop
		while(cur_nb - nb > 1e-9)
		{
			iter--;
			cur_nb *= 0.1;
		}
	}
	else if (nb > 1.0)
	{
		cur_nb = 10.0;

		// loop
		while(nb - cur_nb >= 0.0)
		{
			iter++;
			cur_nb *= 10.0;
		}
	}
	return iter;
}

/*
 * Create (with memory allocation) a vector (length x) of chars
 */
char* get_char_vec(int x)
{
	char *vec;

	vec = (char*) malloc(x*sizeof(char));

	vec = " ";
	
	return vec;
}

/*
 * Free (memory allocation) a vector of chars
 */
void free_char_vec(char *vec)
{
	free(vec);
}

/*
 * Create (with memory allocation) a [x times y] tab of char
 */
char** get_char_tab(int x, int y)
{
	int i;

	char **tab;

	tab = (char**) malloc(x*sizeof(char*));

	for(i=0; i<x; i++)
	{
		tab[i] = (char*) malloc(y*sizeof(char)); 
		tab[i] = " ";
	}

	return tab;
}

/*
 * Free (memory allocation) a [x times y] tab of char
 */
void free_char_tab(char** tab, int x)
{
	int i;

	for(i=0; i<0; i++)
	{
		free(tab[i]);
	}

	free(tab);
}

/*
 * Create (with memory allocation) a vector (length x) of doubles
 */
double* get_double_vec(int x)
{
	int i;

	double *vec;

	vec = (double*) malloc(x*sizeof(double));

	for(i=0; i<x; i++)
	{
		vec[i] = 0.0;
	}

	return vec;
}

/*
 * Free (memory allocation) a vector of doubles
 */
void free_double_vec(double *vec)
{
	free(vec);
}

/*
 * Create (with memory allocation) a [x times y] tab of doubles
 */
double** get_double_tab(int x, int y)
{
	int i, j;

	double **tab;

	tab = (double**) malloc(x*sizeof(double*));

	for(i=0; i<x; i++)
	{
		tab[i] = (double*) malloc(y*sizeof(double));

		for(j=0; j<y; j++)
		{
			tab[i][j] = 0.0;
		}
	}

	return tab;
}

/*
 * Free (memory allocation) a [x times y] tab of doubles
 */
void free_double_tab(double **tab, int x)
{
	int i;

	for(i=0; i<x; i++)
	{
		free(tab[i]);
	}

	free(tab);
}


/*
 * Create (with memory allocation) a vector (length x) of integers
 */
int* get_int_vec(int x)
{
	int i;

	int *vec;

	vec = (int*) malloc(x*sizeof(int));

	for(i=0; i<x; i++)
	{
		vec[i] = 0;
	}

	return vec;
}

/*
 * Free (memory allocation) a vector of integers
 */
void free_int_vec(int *vec)
{
	free(vec);
}

/*
 * Create (with memory allocation) a [x times y] tab of integers
 */
int** get_int_tab(int x, int y)
{
	int i, j;

	int **tab;

	tab = (int**) malloc(x*sizeof(int*));

	for(i=0; i<x; i++)
	{
		tab[i] = (int*) malloc(y*sizeof(int));

		for(j=0; j<y; j++)
		{
			tab[i][j] = 0;
		}
	}

	return tab;
}

/*
 * Free (memory allocation) a [x times y] tab of integers
 */
void free_int_tab(int **tab, int x)
{
	int i;

	for(i=0; i<x; i++)
	{
		free(tab[i]);
	}

	free(tab);
}

