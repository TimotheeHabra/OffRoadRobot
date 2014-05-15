/*
 * Functions used to read 'simu_variables.txt' and 'control_variables.txt'
 *
 * author: Nicolas Van der Noot
 */
#include "read_simu_functions.h"
#include "useful_functions.h"

/*
 * Returns a vector of 4 elements indicating the number of variables of:
 *     0: # NONE
 *     1: # IN
 *     2: # OUT
 *     3: # STRUCT
 */
int* read_nb_var_simu(char *fileName)
{
	// -- Variables declaration -- //

	FILE* file;

	int nb_sscanf;
	int i_simu, i_simu_nb_var;
	int varSize;

	char line[FGETS_MAX_SIZE];
	char varName[STRING_MAX_SIZE], type[STRING_MAX_SIZE];
	char simuName[STRING_MAX_SIZE], other[STRING_MAX_SIZE];

	int *nb_var_simu;

	// -- Start reading -- //

	// open file
	file = fopen(fileName, "r");

	// varibales initialization
	i_simu = -1;
	i_simu_nb_var = 0;

	nb_var_simu = (int*) malloc(4*sizeof(int));

	if (file != NULL)
	{
		while (fgets(line, FGETS_MAX_SIZE, file) != NULL)
        {
        	// detect the current simulation part
        	nb_sscanf = sscanf(line, "# %s", simuName);

	    	if (nb_sscanf == 1)
	    	{
	    		if (i_simu >= 0)
	    		{
	    			nb_var_simu[i_simu] = i_simu_nb_var;
	    		}

	    		if (!strcmp(simuName, "NONE"))
	    		{
	    			i_simu = 0;
	    			i_simu_nb_var = 0;
	    		}
	    		else if (!strcmp(simuName, "IN"))
	    		{
	    			i_simu = 1;
	    			i_simu_nb_var = 0;
	    		}
	    		else if (!strcmp(simuName, "OUT"))
	    		{
	    			i_simu = 2;
	    			i_simu_nb_var = 0;
	    		}
	    		else if (!strcmp(simuName, "STRUCT"))
	    		{
	    			i_simu = 3;
	    			i_simu_nb_var = 0;
	    		}
	    		else
	    		{
					printf("Incorrect part of simu_variables: %s\n", line);
					exit(1);
	    		}
	    	}
	    	// if not a comment or a white line
	    	else if ( (sscanf(line, "// %s", other) != 1) && (sscanf(line, "%% %s", other) != 1) && (sscanf(line, "%s", other) == 1) && (i_simu >= 0) )
	    	{
	    		nb_sscanf = sscanf(line, "%s %s %d", varName, type, &varSize);

            	if (nb_sscanf == 3)
            	{
            		i_simu_nb_var++;
            	}
	    	}
	    }

	    // last structure
	    if (i_simu >= 0)
		{
			nb_var_simu[i_simu] = i_simu_nb_var;
		}

		// close the file
	    fclose(file);

	    return nb_var_simu;
	}
	// could not read the file
    else
    {
    	printf("No file: %s\n", fileName);
    	exit(1);
    }
}

/*
 * Fill 'varNameTab', 'typeTab' and 'varSizeTab'
 * containing the variables names, types and size
 */
void read_simu_variables(char *fileName, int *nb_var_simu, char ***varNameTab, char ***typeTab, int **varSizeTab)
{
	// -- Variables declaration -- //

	FILE* file;

	int nb_sscanf;
	int varSize;
	int i_simu, i_simu_var, i_line;

	char line[FGETS_MAX_SIZE];
	char varName[STRING_MAX_SIZE], type[STRING_MAX_SIZE];
	char simuName[STRING_MAX_SIZE], other[STRING_MAX_SIZE];


	// -- Variables initialization -- //

	i_line = 0;
	i_simu = -1;
	i_simu_var = 0;

	file = fopen(fileName, "r");


	// -- Start reading -- //

	if (file != NULL)
	{
		while (fgets(line, FGETS_MAX_SIZE, file) != NULL)
        {
        	nb_sscanf = sscanf(line, "# %s", simuName);

        	// detect the simulation part
	    	if (nb_sscanf == 1)
	    	{
	    		if (!strcmp(simuName, "NONE"))
	    		{
	    			i_simu = 0;
	    			i_simu_var = 0;
	    		}
	    		else if (!strcmp(simuName, "IN"))
	    		{
	    			i_simu = 1;
	    			i_simu_var = 0;
	    		}
	    		else if (!strcmp(simuName, "OUT"))
	    		{
	    			i_simu = 2;
	    			i_simu_var = 0;
	    		}
	    		else if (!strcmp(simuName, "STRUCT"))
	    		{
	    			i_simu = 3;
	    			i_simu_var = 0;
	    		}
	    		else
	    		{
					printf("Incorrect part of simu_variables: %s\n", line);
					exit(1);
	    		}
	    	}
	    	// not a comment, not a white line
	    	else if ( (sscanf(line, "// %s", other) != 1) && (sscanf(line, "%% %s", other) != 1) && (sscanf(line, "%s", other) == 1) && (i_simu >= 0) )
	    	{
	    		nb_sscanf = sscanf(line, "%s %s %d", varName, type, &varSize);

	    		// correct entry line
	    		if (nb_sscanf == 3)
            	{
            		if (i_simu >= 0)
            		{
            			strcpy(varNameTab[i_simu][i_simu_var], varName);
            			strcpy(typeTab[i_simu][i_simu_var]   , type);

            			varSizeTab[i_simu][i_simu_var] = varSize;            			
            		}
            		else
            		{
            			printf("Not inside a simulation part: %d: %s\n", i_line, line);
            		}
            		
            		i_simu_var++;
            	}            	       	
            	else
        		{
        			printf("Problem with line %d: %s\n", i_line, line);
        		}  
	    	}

	    	i_line++; // move to the next line
	    }

		fclose(file);
	}
    else
    {
    	printf("No file: %s\n", fileName);
    	exit(1);
    }
}
