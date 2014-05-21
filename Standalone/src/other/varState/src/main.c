/*
 * Main file used to generate 
 *     - c files
 *           'user_sf_IO.c', 'user_sf_IO.h', 'ControllersStruct.c' and 'ControllersStruct.h'
 *     - Matlab files
 *           'simu_variables.m' and 'control_variables.m'
 *
 * author: Nicolas Van der Noot
 */		
#include "main_header.h"
#include "cmake_config.h"

/*
 * Generates 'simu_variables.m'
 */
void generate_simu_variables_m()
{
	// variables declaration
	char *fileName, *fileout_simu;

	int *nb_var_simu, **varSizeTab;
	char ***varNameTab, ***typeTab; 

	// files initialization
	fileName     = PROJECT_ABS_PATH"/src/project/varState/simu_variables.txt";
	fileout_simu = PROJECT_ABS_PATH"/../workR/simu_variables.m";

	// variables, vectors and tabulars initialization
	nb_var_simu = read_nb_var_simu(fileName);

	varNameTab = create_triple_char(4, nb_var_simu, STRING_MAX_SIZE);
	typeTab    = create_triple_char(4, nb_var_simu, STRING_MAX_SIZE);
	varSizeTab = create_double_int(4 , nb_var_simu);

	read_simu_variables(fileName, nb_var_simu, varNameTab, typeTab, varSizeTab);

	// generation
	print_matlab_simu_variables(nb_var_simu, varNameTab, typeTab, varSizeTab, fileout_simu);

	// free memory
	free_simu_var(nb_var_simu, varSizeTab, varNameTab, typeTab);
}

/*
 * Generates 'control_variables.m'
 */
void generate_control_variables_m()
{
	// variables declaration
	char *fileName, *fileout_ctrl;

	int nb_ctrl;
	int *nb_var_ctrl, **varSizeTab,  ***twoVarSizeTab;
	char ***varNameTab, ***typeTab, ***strTypeTab, **namesCtrl;

	// files initialization
	fileName     = PROJECT_ABS_PATH"/src/project/varState/control_variables.txt";
	fileout_ctrl = PROJECT_ABS_PATH"/../workR/control_variables.m";

	// variables, vectors and tabulars initialization
	nb_ctrl = read_nb_ctrl(fileName);
	nb_var_ctrl = read_nb_var_ctrl(fileName, nb_ctrl);

	varNameTab = create_triple_char(nb_ctrl, nb_var_ctrl, STRING_MAX_SIZE);
	typeTab    = create_triple_char(nb_ctrl, nb_var_ctrl, STRING_MAX_SIZE);
	strTypeTab = create_triple_char(nb_ctrl, nb_var_ctrl, STRING_MAX_SIZE);
	varSizeTab = create_double_int(nb_ctrl , nb_var_ctrl);
	namesCtrl  = create_double_char(nb_ctrl, STRING_MAX_SIZE);
	twoVarSizeTab = create_triple_int(nb_ctrl, nb_var_ctrl, 2);

	read_ctrl_variables(fileName, nb_ctrl, nb_var_ctrl, varNameTab, typeTab, varSizeTab, twoVarSizeTab, namesCtrl);

	// generation
    print_matlab_control_variables(nb_ctrl, nb_var_ctrl, varNameTab, typeTab, varSizeTab, twoVarSizeTab, namesCtrl, fileout_ctrl);

    // free memory
   	free_ctrl_var(nb_ctrl, nb_var_ctrl, varNameTab, typeTab, namesCtrl, varSizeTab, twoVarSizeTab);
}

/*
 * Generates 'user_sf_IO.c' and 'user_sf_IO.h'
 */
void generate_user_sf_IO()
{
	// variables declaration
	char *fileName, *fileoutC, *fileoutH;

	int *nb_var_simu, **varSizeTab;
	char ***varNameTab, ***typeTab; 

	// files initialization
	fileName = PROJECT_ABS_PATH"/src/project/varState/simu_variables.txt";
	fileoutC = PROJECT_ABS_PATH"/src/project/user_files/user_sf_IO.c";
	fileoutH = PROJECT_ABS_PATH"/src/project/user_files/user_sf_IO.h";

	// variables, vectors and tabulars initialization
	nb_var_simu = read_nb_var_simu(fileName);

	varNameTab = create_triple_char(4, nb_var_simu, STRING_MAX_SIZE);
	typeTab    = create_triple_char(4, nb_var_simu, STRING_MAX_SIZE);
	varSizeTab = create_double_int(4 , nb_var_simu);

	read_simu_variables(fileName, nb_var_simu, varNameTab, typeTab, varSizeTab);

	// generation
	print_c_simu_variables(nb_var_simu, varNameTab, typeTab, varSizeTab, fileoutC, fileoutH);

	// free memory
	free_simu_var(nb_var_simu, varSizeTab, varNameTab, typeTab);
}



/*
 * Generates 'ControllersStruct.c' and 'ControllersStruct.h'
 */
void generate_ControllersStruct()
{
	// variables declaration
	char *fileName, *fileoutC, *fileoutH;

	int nb_ctrl;
	int *nb_var_ctrl, **varSizeTab,  ***twoVarSizeTab;
	char ***varNameTab, ***typeTab, ***strTypeTab, **namesCtrl;

	printf("%s\n", PROJECT_ABS_PATH);

	// files initialization
	fileName = PROJECT_ABS_PATH"/src/project/varState/control_variables.txt";
	fileoutC = PROJECT_ABS_PATH"/src/project/controller_files/ControllersStruct.c";
	fileoutH = PROJECT_ABS_PATH"/src/project/controller_files/ControllersStruct.h";

	// variables, vectors and tabulars initialization
	nb_ctrl = read_nb_ctrl(fileName);
	nb_var_ctrl = read_nb_var_ctrl(fileName, nb_ctrl);

	varNameTab = create_triple_char(nb_ctrl, nb_var_ctrl, STRING_MAX_SIZE);
	typeTab    = create_triple_char(nb_ctrl, nb_var_ctrl, STRING_MAX_SIZE);
	strTypeTab = create_triple_char(nb_ctrl, nb_var_ctrl, STRING_MAX_SIZE);
	varSizeTab = create_double_int(nb_ctrl , nb_var_ctrl);
	namesCtrl  = create_double_char(nb_ctrl, STRING_MAX_SIZE);
	twoVarSizeTab = create_triple_int(nb_ctrl, nb_var_ctrl, 2);

	read_ctrl_variables(fileName, nb_ctrl, nb_var_ctrl, varNameTab, typeTab, varSizeTab, twoVarSizeTab, namesCtrl);

	// generation
    print_c_ctrl_variables(nb_ctrl, nb_var_ctrl, varNameTab, typeTab, varSizeTab, twoVarSizeTab, namesCtrl, fileoutC, fileoutH);

    // free memory
   	free_ctrl_var(nb_ctrl, nb_var_ctrl, varNameTab, typeTab, namesCtrl, varSizeTab, twoVarSizeTab);
}

/*
 * Free memory associated to the simulation structures generation
 */
void free_simu_var(int *nb_var_simu, int **varSizeTab, char ***varNameTab, char ***typeTab)
{
	free_triple_char(varNameTab, 4, nb_var_simu);
	free_triple_char(typeTab   , 4, nb_var_simu);
	free_double_int(varSizeTab , 4);

	free(nb_var_simu);
}

/*
 * Free memory associated to the controllers structures generation
 */
void free_ctrl_var(int nb_ctrl, int *nb_var_ctrl, char ***varNameTab, char ***typeTab,
	char **namesCtrl, int **varSizeTab, int ***twoVarSizeTab)
{
	free_triple_char(varNameTab, nb_ctrl, nb_var_ctrl);
	free_triple_char(typeTab   , nb_ctrl, nb_var_ctrl);
	free_double_int(varSizeTab , nb_ctrl);
	free_double_char(namesCtrl, nb_ctrl);
	free_triple_int(twoVarSizeTab, nb_ctrl, nb_var_ctrl);

	free(nb_var_ctrl);
}
 
int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		printf("Error, missing argument !\n");
	}
	else if (argc > 2)
	{
		printf("Error: only one argument required, but received %d arguments !\n", argc-1);
	}
	else
	{
		if ( !strcmp(argv[1], "simu") )
		{
			generate_user_sf_IO();
		}
		else if ( !strcmp(argv[1], "ctrl") )
		{
			generate_ControllersStruct();
		}
		else if ( !strcmp(argv[1], "simu_sync") )
		{
			generate_simu_variables_m();
		}
		else if ( !strcmp(argv[1], "ctrl_sync") )
		{
			generate_control_variables_m();
		}
		else
		{
			printf("Error: %s is not a good argument !\n", argv[1]);
			printf("Arguments accepted: 'simu', 'ctrl', 'simu_sync' or 'ctrl_sync' (without '')\n");
		}
	}
 
    return 0;
}
