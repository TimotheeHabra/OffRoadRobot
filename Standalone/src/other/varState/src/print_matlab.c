/*
 * Fonctions used to generate the files in Matlab
 * which are the equivalent of 'control_variables.txt' and 'simu_variables.txt'
 *
 * author: Nicolas Van der Noot
 */
#include "print_matlab.h"
#include "useful_functions.h"

/*
 * Generates the file in Matlab which is the equivalent
 * of 'control_variables.txt' 
 */
void print_matlab_control_variables(int nb_ctrl, int *nb_var_ctrl, char ***varNameTab, char ***typeTab, 
	char ***strTypeTab, int **varSizeTab, int ***twoVarSizeTab, char **namesCtrl, char *fileout)
{
	int i, j;

	// File declaration
    FILE* fid           = NULL; // internal filename

    // Opening file
    fid = fopen(fileout, "w"); 

    // Fill the file
    if(fid == NULL)
    {
        printf("error: cannot open file '%s'\n", fileout);
        exit(1);
    }   

    fprintf(fid, "function [controllers_struct] = control_variables()\n\n");
	fprintf(fid, "%% Definition of the control variables\n");
	fprintf(fid, "%% For each line: varname , type , size/str_type\n");
	fprintf(fid, "%%  - varname: name of the variable (with '')\n");
	fprintf(fid, "%%  - type: 'int' / 'double' / 'structure'\n");
	fprintf(fid, "%%      if type == 'int' / 'double'\n");
	fprintf(fid, "%%          - size/str_type: number of elements in the vector\n");
	fprintf(fid, "%%               1    :  simple variable\n");
	fprintf(fid, "%%               n    :  vector of n (n>1) elements\n");
	fprintf(fid, "%%               [m n]:  tabular of 2 entries with a size m*n \n");
	fprintf(fid, "%%      if type == 'structure' \n");
	fprintf(fid, "%%          - size/str_type: the type of the structure (its name with '')\n\n");

	for(i=0; i<nb_ctrl; i++) // loop on all the controllers
	{
		fprintf(fid, "%%%% controller_%d\n\n", i+1);
		fprintf(fid, "controller_%d_name = '%s';\n", i+1, namesCtrl[i]);
		fprintf(fid, "controller_%d_vars = {\n", i+1);

		for(j=0; j<nb_var_ctrl[i]; j++) // loop on all this controller variables
		{
			fprintf(fid, "    '%s',", varNameTab[i][j]);
    		fprintf(fid, "'%s',", typeTab[i][j]);

    		if (varSizeTab[i][j] >= 0) // vector
    		{
    			fprintf(fid, "%d\n", varSizeTab[i][j]);
    		}
    		else if (varSizeTab[i][j] == -1) // structure
    		{
    			fprintf(fid, "'%s'\n", strTypeTab[i][j]);
    		}	
    		else if (varSizeTab[i][j] == -2) // 2-entries tabular
    		{
    			fprintf(fid, "[%d %d]\n", twoVarSizeTab[i][j][0], twoVarSizeTab[i][j][1]);
    		} 
		}

		fprintf(fid, "};\n\n\n");
	}

	fprintf(fid, "%%%% controllers_vars\n\n");
	fprintf(fid, "controllers_struct = {\n");

	for(i=0; i<nb_ctrl; i++) // loop on all the controllers
	{
		fprintf(fid, "    controller_%d_name, controller_%d_vars;\n", i+1, i+1);
	}

	fprintf(fid, "};\n\n");
	fprintf(fid, "end\n");

	fclose(fid);
	printf(">> 'control_variables.m' created\n");
}

/*
 * Generates the file in Matlab which is the equivalent
 * of 'simu_variables.txt' 
 */
void print_matlab_simu_variables(int *nb_var_simu, char ***varNameTab, char ***typeTab, int **varSizeTab, char *fileout)
{
	int i, j;

	// File declaration
    FILE* fid           = NULL; // internal filename

    // Opening file
    fid = fopen(fileout, "w"); 

    // Fill the file
    if(fid == NULL)
    {
        printf("error: cannot open file '%s'\n", fileout);
        exit(1);
    }   

    fprintf(fid, "function [simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct] = simu_variables()\n\n");
	fprintf(fid, "%% Definition of the simulation variables and I/O ports\n");
	fprintf(fid, "%% For each line: varname , type , size/str_type\n");
	fprintf(fid, "%% 4 types of simulation variables (simu_vars_none,...)\n");
	fprintf(fid, "%%       . none = internal variable (type: 'int'/'double')\n");
	fprintf(fid, "%%       . in = input (type: 'int'/'double')\n");
	fprintf(fid, "%%       . out = output (type: 'int'/'double')\n");
	fprintf(fid, "%%       . struct = structure variable (type: structure name with '')\n");
	fprintf(fid, "%%   - varname = name of the port or of the variable (with '')\n");
	fprintf(fid, "%%   - size = number of elements in the vector\n");
	fprintf(fid, "%%               (put '1' for 'simu_vars_struct')\n");
	fprintf(fid, "%%           (1: simple varibale,  >1: vector -> do not use the 0 index)\n\n");

	for(i=0; i<4; i++) // loop on all the simu parts
	{
		switch(i)
		{
		    case 0:
		       fprintf(fid, "simu_vars_none = {\n");
		       break;

		    case 1:
		       fprintf(fid, "simu_vars_in = {\n");
		       break;

		    case 2:
		       fprintf(fid, "simu_vars_out = {\n");
		       break;

		    case 3:
		       fprintf(fid, "simu_vars_struct = {\n");
		       break; 
		  
		    default : 
		       break;
		}

		for(j=0; j<nb_var_simu[i]; j++) // loop on all the varaibles of this simu part
		{
			fprintf(fid, "    '%s','%s',%d\n", varNameTab[i][j], typeTab[i][j], varSizeTab[i][j]);
		}

		fprintf(fid, "};\n\n");
	}

	fprintf(fid, "end\n");

	fclose(fid);
	printf(">> 'simu_variables.m' created\n");
}
