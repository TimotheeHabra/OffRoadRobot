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

double sign(double a) { return (a<0 ? -1 : 1); }

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
 * Copy vec1 into vec2, the 2 vec are [x] sized vec of doubles
 */
void copy_double_vec(double *vec1, double *vec2, int x)
{
	int i; 
	for(i=0; i<x; i++)
	{
		vec2[i] =vec1[i];
	}
}

/*
 * Print vec in console; vec is a vector (length x) of doubles
 */
void print_double_vec(double *vec, int x)
{
	int i; 
	printf("[");
	for(i=0; i<x; i++)
	{
		printf("%f ",vec[i]);
	}
	printf("]\n");
}
void save_double_vec(double *vec, int x,char *name)
{
	FILE *file_out;
	int i; 
	// File declaration
    file_out = NULL; // internal filename

    // Opening file
    file_out = fopen(name,"wt"); 

    // Fill the file
    if(file_out == NULL)
    {
        printf("error: cannot open file '%s'\n", name);
        exit(1);
    } 
		
	for(i=0; i<x; i++)
	{
			fprintf(file_out,"\t %f ",vec[i]);
	}

	fclose(file_out);
}

// return 1 if any elment is non zero , 0 otherwise
int any_double_vec(double *vec, int x)
{
	int i; 
	int any=0;
	for(i=0; i<x; i++)
	{
		if(vec[i] != 0.0)
		{
			any =1;
			return any;
		}
	}
	return any;
}
void zeros_double_vec(double *vec, int x)
{
	int i; 
	for(i=0; i<x; i++)
	{
		vec[i] = 0.0;
	}
}
double norm_double_vec(double *vec, int x)
{
	int i; 
	double norm_2 = 0.0;
	for(i=0; i<x; i++)
	{
		norm_2 =norm_2 + (vec[i]*vec[i]);
	}
	return sqrt(norm_2);
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
 * Copy tab1 into tab2, the 2 vec are [x times y] sized vec of doubles
 */
void copy_double_tab(double **tab1, double **tab2, int x , int y)
{
	int i,j; 
	for(i=0; i<x; i++)
	{
		for(j=0; j<y; j++)
		{
			tab2[i][j] =tab1[i][j];
		}
	}
}

/*
 * Print tab in console; vec is [x times y] sized vec of doubles
 */
void print_double_tab(double **tab, int x , int y)
{
	int i,j; 
	for(i=0; i<x; i++)
	{
		printf("|");
		for(j=0; j<y; j++)
		{
			printf("%f ",tab[i][j]);
		}
		printf("|\n");
	}
}

void slct_double_vec(double* vec1, int x1, int* vec2, int x2, double* vec3)
{
	int i; 
	for(i=0; i<x2; i++)
	{
		vec3[i] = vec1[vec2[i]];
	}
}
void slctc_double_tab(double **tab1, int x1, int y1, double **tab2, int y2, int* vec)
{
	int i,j; 
	for(i=0; i<y2; i++)
	{
		for(j=0; j<x1; j++)
		{
			tab2[j][i] = tab1[j][vec[i]];
		}
	}
}
void slctr_double_tab(double **tab1, int x1, int y1, double **tab2, int x2, int* vec)
{
	int i,j; 
	for(i=0; i<x2; i++)
	{
		for(j=0; j<y1; j++)
		{
			tab2[i][j] = tab1[vec[i]][j];
		}
	}
}
void zeros_double_tab(double **tab, int x, int y)
{
	int i,j; 
	for(i=0; i<x; i++)
	{
		for(j=0; j<y; j++)
		{
			tab[i][j] = 0.0;
		}
	}

}
void save_double_tab(double **tab, int x, int y,char *name)
{
	FILE *file_out;
	int i,j; 
	// File declaration
    file_out = NULL; // internal filename

    // Opening file
    file_out = fopen(name,"wt"); 

    // Fill the file
    if(file_out == NULL)
    {
        printf("error: cannot open file '%s'\n", name);
        exit(1);
    } 
		
	for(i=0; i<x; i++)
	{
		for(j=0; j<y; j++)
		{
			fprintf(file_out,"\t %f ",tab[i][j]);
		}
		fprintf(file_out,"\n");
	}

	fclose(file_out);
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
 * Copy vec1 into vec2, the 2 vec are [x] sized vec of ints
 */
void copy_int_vec(int *vec1, int *vec2, int x)
{
	int i; 
	for(i=0; i<x; i++)
	{
		vec2[i] =vec1[i];
	}
}

/*
 * Print vec in console; vec is a vector (length x) of ints
 */
void print_int_vec(int *vec, int x)
{
	int i; 
	printf("[");
	for(i=0; i<x; i++)
	{
		printf("%d ",vec[i]);
	}
	printf("]\n");
}

/*
 * Sort vec1 into vec2, the 2 vec are [x] sized vec of ints
 */
void sort_int_vec(int *vec1, int *vec2, int x)
{
	int i, j; 
	int threshold = -INT_MAX;
	int val = INT_MAX;
	int n = 0;
	for(i=0; i<x; )
	{
		n = 0;
		val = INT_MAX;
		for(j=0; j<x; j++)
		{
			if(vec1[j]>threshold)
			{
				if (val > vec1[j])
				{
					n=1;
					val = vec1[j];
				}
				else if (val == vec1[j])
				{
					n++;
				}
			}
		}
		while(n != 0)
		{
		vec2[i] = val;
		i++;
		n--;
		}
		threshold = val;
	}
}


/*
 * Fill vec as [0 1 2 3 ... x] , with vec [x] sized vec of ints
 */
void f0123_int_vec(int* vec, int x)
{
	int i; 
	for(i=0; i<x; i++)
	{
		vec[i] = i;
	}
}

/*
 * Concat vec1 and vec 2 into vec3, the vec1 and vec2 are [x1] and [x2] sized vec of ints
 */
void conc_int_vec(int* vec1, int x1, int* vec2, int x2, int* vec3)
{
	int i; 
	for(i=0; i<x1; i++)
	{
		vec3[i] = vec1[i];
	}
	for(i=0; i<x2; i++)
	{
		vec3[x1+i] = vec2[i];
	}
}

void slct_int_vec(int* vec1, int x1, int* vec2, int x2, int* vec3)
{
	int i; 
	for(i=0; i<x2; i++)
	{
		vec3[i] = vec1[vec2[i]];
	}
}
int find_int_vec(int* vec, int x, int f)
{
	int i;
	for(i=0; i<x; i++)
	{
		if (vec[i] == f)
		{
			return i;
		}
	}
	return -1;
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

/*
 * Copy tab1 into tab2, the 2 vec are [x times y] sized vec of ints
 */
void copy_int_tab(int **tab1, int **tab2, int x , int y)
{
	int i,j; 
	for(i=0; i<x; i++)
	{
		for(j=0; j<y; j++)
		{
			tab2[i][j] =tab1[i][j];
		}
	}
}

/*
 * Print tab in console; vec is [x times y] sized vec of ints
 */
void print_int_tab(int **tab, int x , int y)
{
	int i,j; 
	for(i=0; i<x; i++)
	{
		printf("|");
		for(j=0; j<y; j++)
		{
			printf("%d ",tab[i][j]);
		}
		printf("|\n");
	}
}