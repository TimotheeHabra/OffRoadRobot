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


/*
 * Print an empty file (just the generation date is indicated)
 */
void print_empty_file(char *fileout)
{	
	FILE *fid;

	// File declaration
    fid = NULL; 

    // Opening file
    fid = fopen(fileout, "wt");

    if(fid == NULL)
    {
        printf("error: cannot open file '%s'\n", fileout);
        exit(1);
    }  

    fprintf(fid, "//---------------------------\n");
	fprintf(fid, "// Last update : %s", get_time_machine());
	fprintf(fid, "//---------------------------\n\n");

	fclose(fid);
}


/*
 * Generates 'UserModelStruct.c' and 'UserModelStruct.h'
 */
void generate_user_models()
{
	// variables declaration
    char *fileoutC, *fileoutH; //*fileoutCheck;

    struct stat mbs_stat, outC_stat, outH_stat;

    MDS_gen_strct* gen = NULL;

	// files initialization
	fileoutC     = PROJECT_ABS_PATH"/src/project/user_files/UserModelStruct.c";
	fileoutH     = PROJECT_ABS_PATH"/src/project/user_files/UserModelStruct.h";

	// print check empty file
	//print_empty_file(fileoutCheck);

    if (stat(MBS_FILE, &mbs_stat))
    {
        printf("Problem: could not read the creation date of %s !\n", MBS_FILE);
        return;
    }

    if (stat(fileoutC, &outC_stat))
    {
        printf("Problem: could not read the creation date of %s !\n", fileoutC);
        return;
    }

    if (stat(fileoutH, &outH_stat))
    {
        printf("Problem: could not read the creation date of %s !\n", fileoutH);
        return;
    }

    if ( (mbs_stat.st_mtime >= outC_stat.st_mtime) || (mbs_stat.st_mtime >= outH_stat.st_mtime) )
    {
        gen =  MDS_mbs_reader(MBS_FILE);
        print_c_user_models(gen, fileoutC, fileoutH);
    }

    //free_MDS_gen_strct(gen);
}

/*
 * Generates 'user_all_id.h' 
 */
void generate_user_all_id()
{
	// variables declaration
    char *fileoutH; //, *fileoutCheck;

    struct stat mbs_stat, outH_stat;

	MDS_gen_strct* gen;
	gen = NULL;

	// files initialization
	fileoutH     = PROJECT_ABS_PATH"/src/project/user_files/user_all_id.h";
	//fileoutCheck = PROJECT_ABS_PATH"/src/project/user_files/update_check/user_all_id_check.txt";

	// print check empty file
	//print_empty_file(fileoutCheck);

    if (stat(MBS_FILE, &mbs_stat))
	{
        printf("Problem: could not read the creation date of %s !\n", MBS_FILE);
		return;
	}

	if (stat(fileoutH, &outH_stat)) 
	{
		printf("Problem: could not read the creation date of %s !\n", fileoutH);
		return;
	}

    if (mbs_stat.st_mtime >= outH_stat.st_mtime)
	{
		gen =  MDS_mbs_reader(MBS_FILE);
		print_c_user_all_id(gen, fileoutH);
	}
	//free_MDS_gen_strct(gen);
}
 
int main(int argc, char *argv[])
{
	if (argc > 2)
	{
		printf("Error: only one argument required, but received %d arguments !\n", argc-1);
	}
	else
	{
		if ( !strcmp(argv[1], "all") )
		{
			generate_user_models();
			generate_user_all_id();
		}
		else if ( !strcmp(argv[1], "user_models") )
		{
			generate_user_models();
		}
		else if ( !strcmp(argv[1], "user_all_id") )
		{
			generate_user_all_id();
		}
		else
		{
			printf("Error: %s is not a good argument !\n", argv[1]);
			printf("Arguments accepted: 'user_models' and 'user_all_id' (without '')\n");
		}
	}
    return 0;
}
