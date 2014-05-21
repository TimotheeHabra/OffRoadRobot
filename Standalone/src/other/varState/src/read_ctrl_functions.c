/*
 * Functions used to read 'control_variables.txt'
 *
 * author: Nicolas Van der Noot
 */
#include "read_ctrl_functions.h"
#include "useful_functions.h"

/*
 * Returns the number of controllers defined by the user
 */
int read_nb_ctrl(char *fileName)
{
	// -- Variables declaration -- //

	FILE* file;

	int nb_ctrl;
	int nb_sscanf;

	char line[FGETS_MAX_SIZE], ctrlName[STRING_MAX_SIZE];

	file = fopen(fileName, "r");

	nb_ctrl = 0;

	// -- Start reading -- //

	if (file != NULL)
	{
		while (fgets(line, FGETS_MAX_SIZE, file) != NULL)
        {
	    	nb_sscanf = sscanf(line, "# %s", ctrlName);

	    	// controller detected
	    	if (nb_sscanf == 1)
	    	{
	    		nb_ctrl++;
	    	}
	    }

		fclose(file);

		return nb_ctrl;
	}
	// could not open the file
    else
    {
    	printf("No file: %s\n", fileName);
    	exit(1);
    }
}

/*
 * Returns a vector of nb_ctrl elements indicating 
 * the number of variables of each of the nb_ctrl controllers
 */
int* read_nb_var_ctrl(char *fileName, int nb_ctrl)
{
	// -- Variables declaration -- //

	FILE* file;

	int i_ctrl, i_ctrl_nb_var;
	int varSize, nb_sscanf, varSize1, varSize2;

	int *nb_var_ctrl;

	char line[FGETS_MAX_SIZE];
	char varName[STRING_MAX_SIZE], type[STRING_MAX_SIZE], ctrlName[STRING_MAX_SIZE];
    char other[STRING_MAX_SIZE];


    // -- Initialization -- //

	file = fopen(fileName, "r");

	if (nb_ctrl > 0)
    {
    	nb_var_ctrl = (int*) malloc(nb_ctrl*sizeof(int));
    }
    else
    {
    	printf("No controller defined in this file !\n");
    	exit(1);
    }

	i_ctrl = -1;
	i_ctrl_nb_var = 0;


	// -- Start reading -- //

	if (file != NULL)
	{
		while (fgets(line, FGETS_MAX_SIZE, file) != NULL)
        {
        	// controller declaration
        	nb_sscanf = sscanf(line, "# %s", ctrlName);

	    	if (nb_sscanf == 1)
	    	{
	    		if (i_ctrl >= 0)
	    		{
	    			if (!i_ctrl_nb_var)
	    			{
	    				printf("Controller defined with no variable !\n");
	    				exit(1);
	    			}
	    			nb_var_ctrl[i_ctrl] = i_ctrl_nb_var;
	    		}
	    		i_ctrl++;
	    		i_ctrl_nb_var = 0;
	    	}
	    	// not a comment, not a white line
	    	else if ( (sscanf(line, "// %s", other) != 1) && (sscanf(line, "%% %s", other) != 1) && (sscanf(line, "%s", other) == 1) && (i_ctrl >= 0) )
	    	{
	    		nb_sscanf = sscanf(line, "%s %s %d", varName, type, &varSize);

	    		// standard entry
            	if (nb_sscanf == 3)
            	{
            		i_ctrl_nb_var++;
            	}
            	// tabular of 2 entries
            	else if ( nb_sscanf == 2 )
            	{
            		nb_sscanf = sscanf(line, "%s %s [%d %d]", varName, type, &varSize1, &varSize2);
            		if (nb_sscanf == 4)
	            	{
	            		i_ctrl_nb_var++;
	            	}
            	}
	    	}
	    }

	    // last element
	    if (i_ctrl >= 0)
		{
			nb_var_ctrl[i_ctrl] = i_ctrl_nb_var;
		}

	    fclose(file);

	    return nb_var_ctrl;
	}
	// could nopt open the file
    else
    {
    	printf("No file: %s\n", fileName);
    	exit(1);
    }
}

/*
 * Fill 'varNameTab', 'typeTab', 'varSizeTab', 'twoVarSizeTab' and 'namesCtrl'
 * containing the variables names, types, String type, size, size (in case of two-entries tabular) and the nemes of the controllers
 */
void read_ctrl_variables(char *fileName, int nb_ctrl, int *nb_var_ctrl, char ***varNameTab, char ***typeTab, int **varSizeTab, int ***twoVarSizeTab, char **namesCtrl)
{
	// -- Variables declaration -- //

	FILE* file;

	int i_ctrl, i_line, i_ctrl_var;
	int nb_sscanf, varSize, varSize1, varSize2;

	char line[FGETS_MAX_SIZE];
	char varName[STRING_MAX_SIZE], type[STRING_MAX_SIZE], ctrlName[STRING_MAX_SIZE];
    char other[STRING_MAX_SIZE];


    // -- Initiaization -- //

	file = fopen(fileName, "r");

	if (nb_ctrl > 0)
    {
    	nb_var_ctrl = (int*) malloc(nb_ctrl*sizeof(int));
    }
    else
    {
    	printf("No controller defined in this file !\n");
    	exit(1);
    }

	i_ctrl = -1;
	i_line = 0;
	i_ctrl_var = 0;

	
	// -- Start reading -- //

	if (file != NULL)
	{
		while (fgets(line, FGETS_MAX_SIZE, file) != NULL)
        {
        	// controller declaration
        	nb_sscanf = sscanf(line, "# %s", ctrlName);

	    	if (nb_sscanf == 1)
	    	{
	    		i_ctrl++;
	    		i_ctrl_var = 0;
	    		
	    		strcpy(namesCtrl[i_ctrl], ctrlName);
	    	}
	    	// not a comment, not a white line
	    	else if ( (sscanf(line, "// %s", other) != 1) && (sscanf(line, "%% %s", other) != 1) && (sscanf(line, "%s", other) == 1) && (i_ctrl >= 0) )
	    	{
	    		nb_sscanf = sscanf(line, "%s %s %d", varName, type, &varSize);

	    		// standard entry
	    		if (nb_sscanf == 3)
            	{
            		if (i_ctrl >= 0)
            		{
            			strcpy(varNameTab[i_ctrl][i_ctrl_var], varName);
            			strcpy(typeTab[i_ctrl][i_ctrl_var]   , type);

            			varSizeTab[i_ctrl][i_ctrl_var] = varSize;            			
            		}
            		else
            		{
            			printf("Not inside a controller: %d: %s\n", i_line, line);
            		}
            		
            		i_ctrl_var++;
            	}
            	// 2-entries tabular
            	else if ( nb_sscanf == 2 )
            	{
            		nb_sscanf = sscanf(line, "%s %s [%d %d]", varName, type, &varSize1, &varSize2);

            		if (nb_sscanf == 4)
	            	{
	            		if (i_ctrl >= 0)
	            		{
	            			strcpy(varNameTab[i_ctrl][i_ctrl_var], varName);
	            			strcpy(typeTab[i_ctrl][i_ctrl_var]   , type);

	            			twoVarSizeTab[i_ctrl][i_ctrl_var][0] = varSize1;
	            			twoVarSizeTab[i_ctrl][i_ctrl_var][1] = varSize2;
	            			varSizeTab[i_ctrl][i_ctrl_var] = 0;
	            		}
	            		else
	            		{
	            			printf("Not inside a controller: %d: %s\n", i_line, line);
	            		}
	            		
	            		i_ctrl_var++;
	            	}
            	}           	
            	else
        		{
        			printf("Problem with line %d: %s\n", i_line, line);
        		}  
	    	}

	    	// next line
	    	i_line++;
	    }

	    // close the file to read
		fclose(file);
	}
	// could not read the file
    else
    {
    	printf("No file: %s\n", fileName);
    	exit(1);
    }
}
