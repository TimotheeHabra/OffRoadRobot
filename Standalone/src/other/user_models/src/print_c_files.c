/*
 * Functions used to generate 'user_models.c' 'user_models.h'
 *
 * author: Aubain VERLE
 */
#include "print_c_files.h"

#include <time.h>
#include <string.h>



/*
 * Returns the current time as a String
 *
 * source: http://en.wikipedia.org/wiki/C_date_and_time_functions
 */
char* get_time_machine()
{
	time_t cur_t;
    char* t_string;
 
    // obtain current time as seconds elapsed since the Epoch
    cur_t = time(NULL);
 
    if (cur_t == ((time_t)-1))
    {
        (void) fprintf(stderr, "Failure to compute the current time.");
        return NULL;
    }
 
    // convert to local time format
    t_string = ctime(&cur_t);
 
    if (t_string == NULL)
    {
        (void) fprintf(stderr, "Failure to convert the current time.");
        return NULL;
    }

    return t_string;
}

/*
 * Generates 'UserModelStruct.c' and 'UserModelStruct.h'
 */
void print_c_user_models(MDS_gen_strct* gen, char *fileoutC, char *fileoutH)
{
	int i, j, k;

	FILE *fidC, *fidH;

	// File declaration
    fidC = NULL; // internal filename

    // Opening file
    fidC = fopen(fileoutC, "wt"); 

    // Fill the file
    if(fidC == NULL)
    {
        printf("error: cannot open file '%s'\n", fileoutC);
        exit(1);
    }  

    // File declaration
    fidH = NULL; // internal filename

    // Opening file
    fidH = fopen(fileoutH, "wt"); 

    // Fill the file
    if(fidH == NULL)
    {
        printf("error: cannot open file '%s'\n", fileoutH);
        exit(1);
    }  

    // -- UserModelStruct.h -- //

    fprintf(fidH, "//---------------------------\n");
	fprintf(fidH, "// C-code automatically generated from Gen_mds_user project\n");
	fprintf(fidH, "//\n");
	fprintf(fidH, "//\n");
	fprintf(fidH, "// Last update : %s", get_time_machine());
	fprintf(fidH, "//---------------------------\n\n");
	fprintf(fidH, "\n\n");
	fprintf(fidH,"#ifndef USERMODELSTRUCT_h\n");
	fprintf(fidH,"#define USERMODELSTRUCT_h\n");
	fprintf(fidH,"\n");
	fprintf(fidH,"#include \"lut.h\"\n");
	fprintf(fidH,"#include \"useful_functions.h\"\n");
	fprintf(fidH,"#include \"mbs_xml_reader.h\"\n");
	fprintf(fidH,"\n");
	fprintf(fidH,"// ============================================================ //\n\n");
	fprintf(fidH,"\n");
	fprintf(fidH,"typedef struct UserModelStruct \n");
	fprintf(fidH,"{\n");
	
	for(i=0; i< gen->user_models->n_user_model; i++)
	{
		fprintf(fidH,"    struct %s{\n", gen->user_models->user_model_list[i]->name);
		for(j=0; j< gen->user_models->user_model_list[i]->n_parameter; j++)
		{
			switch( gen->user_models->user_model_list[i]->parameter_list[j]->type)
			{
				case 1:
					fprintf(fidH,"        double %s;\n",gen->user_models->user_model_list[i]->parameter_list[j]->name);
					break;
				case 2:
					fprintf(fidH,"        double* %s;\n",gen->user_models->user_model_list[i]->parameter_list[j]->name);
					break;
				case 3:
					fprintf(fidH,"        char* %s;\n",gen->user_models->user_model_list[i]->parameter_list[j]->name);
					fprintf(fidH,"        // print_c_files.c uncompleted for lut1D\n");
					break;
				case 4:
					fprintf(fidH,"        char* %s;\n",gen->user_models->user_model_list[i]->parameter_list[j]->name);
					fprintf(fidH,"        // print_c_files.c uncompleted for lut2D\n");
					break;
				case 5:
					fprintf(fidH,"        int* %s;\n",gen->user_models->user_model_list[i]->parameter_list[j]->name);
					fprintf(fidH,"        // index of the corresponding values in MBSdataStruct->ux/uxd/ux0\n");
					break;
			}		
		}
		fprintf(fidH,"    } %s;\n", gen->user_models->user_model_list[i]->name);
		fprintf(fidH," \n");
	}
	
	fprintf(fidH,"} UserModelStruct;\n");
	fprintf(fidH,"\n");
	fprintf(fidH,"UserModelStruct* init_UserModelStruct();\n");
	fprintf(fidH,"void free_UserModelStruct(UserModelStruct* ums);\n");
	fprintf(fidH,"void load_UserModelStruct(MDS_gen_strct* gen, UserModelStruct* ums);\n");

	fprintf(fidH,"// ============================================================ //\n \n");
	fprintf(fidH,"# endif""\n");
	fclose(fidH);
	printf(">> 'UserModelStruct.h' created\n");



    // -- UserModelStruct.c -- //

    fprintf(fidC, "//---------------------------\n");
	fprintf(fidC, "// C-code automatically generated from Gen_mds_user project\n");
	fprintf(fidC, "//\n");
	fprintf(fidC, "//\n");
	fprintf(fidC, "// Last update : %s", get_time_machine());
	fprintf(fidC, "//---------------------------\n\n");
	fprintf(fidC, "\n\n");
	fprintf(fidC,"\n");
	fprintf(fidC,"#include \"UserModelStruct.h\"\n");
	fprintf(fidC,"\n");
	fprintf(fidC,"// ============================================================ //\n\n");
	fprintf(fidC,"\n");
	fprintf(fidC,"UserModelStruct* init_UserModelStruct() \n");
	fprintf(fidC,"{\n");
	fprintf(fidC,"    UserModelStruct* ums;\n");
	fprintf(fidC,"    ums = (UserModelStruct*)malloc(sizeof(UserModelStruct));\n");


	for(i=0; i< gen->user_models->n_user_model; i++)
	{
		for(j=0; j< gen->user_models->user_model_list[i]->n_parameter; j++)
		{
			switch( gen->user_models->user_model_list[i]->parameter_list[j]->type)
			{
				case 1:
					fprintf(fidC,"    ums->%s.%s = 0.0;\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name);
					break;
				case 2:
					fprintf(fidC,"    ums->%s.%s = get_double_vec(%d+1);\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name,gen->user_models->user_model_list[i]->parameter_list[j]->n_value);
					fprintf(fidC,"    ums->%s.%s[0] = %d+1;\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name,gen->user_models->user_model_list[i]->parameter_list[j]->n_value);
					break;
				case 3:
					fprintf(fidC,"    ums->%s.%s = NULL;\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name);
					fprintf(fidC,"    // print_c_files.c uncompleted for lut1D\n");
					break;
				case 4:
					fprintf(fidC,"    ums->%s.%s = NULL;\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name);
					fprintf(fidC,"    // print_c_files.c uncompleted for lut2D\n");
					break;
				case 5:
					fprintf(fidC,"    ums->%s.%s = get_int_vec(%d+1);\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name,gen->user_models->user_model_list[i]->parameter_list[j]->n_value);
					fprintf(fidC,"    ums->%s.%s[0] = %d+1;\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name,gen->user_models->user_model_list[i]->parameter_list[j]->n_value);
					break;
			}		
		}
		fprintf(fidC," \n");
	}
	fprintf(fidC,"    return ums;\n");
	fprintf(fidC,"}\n");
	fprintf(fidC,"\n");

	fprintf(fidC,"void free_UserModelStruct(UserModelStruct* ums) \n");
	fprintf(fidC,"{\n");

	for(i=0; i< gen->user_models->n_user_model; i++)
	{
		for(j=0; j< gen->user_models->user_model_list[i]->n_parameter; j++)
		{
			switch( gen->user_models->user_model_list[i]->parameter_list[j]->type)
			{
				case 1:
					break;
				case 2:
					fprintf(fidC,"    free_double_vec(ums->%s.%s);\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name,gen->user_models->user_model_list[i]->parameter_list[j]->n_value);
					break;
				case 3:
				case 4:
					break;
				case 5:
					fprintf(fidC,"    free_int_vec(ums->%s.%s);\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name,gen->user_models->user_model_list[i]->parameter_list[j]->n_value);
					break;
			}		
		}
	}
	fprintf(fidC,"    free(ums);\n");
	fprintf(fidC,"}\n");
	fprintf(fidC,"\n");
	fprintf(fidC," void load_UserModelStruct(MDS_gen_strct* gen, UserModelStruct* ums) \n");
	fprintf(fidC,"{\n");
	fprintf(fidC,"\n");
	fprintf(fidC,"    int ind;\n");
	fprintf(fidC,"    int ind_state_value = 0;\n");
	fprintf(fidC,"\n");

	for(i=0; i< gen->user_models->n_user_model; i++)
	{
		for(j=0; j< gen->user_models->user_model_list[i]->n_parameter; j++)
		{
			switch( gen->user_models->user_model_list[i]->parameter_list[j]->type)
			{
				case 1:
					fprintf(fidC,"    ums->%s.%s = gen->user_models->user_model_list[%d]->parameter_list[%d]->value_list[0];\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name,i,j);
					break;
				case 2:
					fprintf(fidC,"    for(ind=0; ind<gen->user_models->user_model_list[%d]->parameter_list[%d]->n_value; ind++)\n");
					fprintf(fidC,"    {\n");
					fprintf(fidC,"        ums->%s.%s[ind] = gen->user_models->user_model_list[%d]->parameter_list[%d]->value_list[ind];\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name, i, j);
					fprintf(fidC,"    }\n");
					break;
				case 3:
					fprintf(fidC,"    //ums->%s.%s = gen->user_models->user_model_list[i]->parameter_list[j]->lut_ref;\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name);
					fprintf(fidC,"    // print_c_files.c uncompleted for lut1D\n");
					break;
				case 4:
					fprintf(fidC,"    //ums->%s.%s = gen->user_models->user_model_list[i]->parameter_list[j]->lut_ref;\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name);
					fprintf(fidC,"    // print_c_files.c uncompleted for lut2D\n");
					break;
				case 5:
					fprintf(fidC,"    for(ind=0; ind<gen->user_models->user_model_list[%d]->parameter_list[%d]->n_value; ind++)\n",i,j);
					fprintf(fidC,"    {\n");
					fprintf(fidC,"        ums->%s.%s[ind] = ind_state_value;\n",gen->user_models->user_model_list[i]->name , gen->user_models->user_model_list[i]->parameter_list[j]->name);	
					fprintf(fidC,"        ind_state_value++;\n");
					fprintf(fidC,"    }\n");
					break;
			}		
		}
		fprintf(fidC," \n");
	}
	fprintf(fidC,"}\n");
	fprintf(fidC,"\n");

	fprintf(fidC,"// ============================================================ //\n \n");
	fclose(fidC);
	printf(">> 'UserModelStruct.c' created\n");
}

/*
 * Generates 'user_all_id.h'
 */
void print_c_user_all_id(MDS_gen_strct* gen, char *fileoutH)
{
	int i, j, k;
	int ind;

	FILE  *fidH;

    // File declaration
    fidH = NULL; // internal filename

    // Opening file
    fidH = fopen(fileoutH, "wt"); 

    // Fill the file
    if(fidH == NULL)
    {
        printf("error: cannot open file '%s'\n", fileoutH);
        exit(1);
    }  

    // -- user_all_id.h -- //

    fprintf(fidH, "//---------------------------\n");
	fprintf(fidH, "// C-code automatically generated from Gen_mds_user project\n");
	fprintf(fidH, "//\n");
	fprintf(fidH, "//\n");
	fprintf(fidH, "// Last update : %s", get_time_machine());
	fprintf(fidH, "//---------------------------\n\n");
	fprintf(fidH, "\n\n");
	fprintf(fidH,"#ifndef USER_ALL_ID_h\n");
	fprintf(fidH,"#define USER_ALL_ID_h\n");
	fprintf(fidH,"\n");
	fprintf(fidH,"// ============================================================ //\n\n");
	fprintf(fidH,"\n");
	
	fprintf(fidH,"// joint\n");
	fprintf(fidH,"\n");
	for(i=0; i< gen->bodytree->n_joint; i++)
	{
		fprintf(fidH,"#define %s_id %d\n", gen->bodytree->joint_list[i]->name, i);
	}
	fprintf(fidH,"\n\n");

	fprintf(fidH,"// body\n");
	fprintf(fidH,"\n");
	ind=-1;
	for(i=0; i< gen->bodytree->n_body; i++)
	{
		ind += gen->bodytree->body_list[i]->n_joint;
		fprintf(fidH,"#define %s_id %d\n", gen->bodytree->body_list[i]->name, ind);
	}
	fprintf(fidH,"\n\n");

	fprintf(fidH,"// point\n");
	fprintf(fidH,"\n");
	for(i=0; i< gen->n_point; i++)
	{
		if(strcmp(gen->point_list[i]->name,"origin"))
		{
			fprintf(fidH,"#define %s_id %d\n", gen->point_list[i]->name, i);
		}
		else
		{
			fprintf(fidH,"//#define %s_id %d\n", gen->point_list[i]->name, i);
		}
	}
	fprintf(fidH,"\n\n");

/*	fprintf(fidH,"// ball\n");
	fprintf(fidH,"\n");
	for(i=0; i< gen->cuts->n_ball; i++)
	{
		fprintf(fidH,"#define %s_id %d\n", gen->cuts->ball_list[i]->name, i);
	}
	fprintf(fidH,"\n\n");

	fprintf(fidH,"// rod\n");
	fprintf(fidH,"\n");
	for(i=0; i< gen->cuts->n_rod; i++)
	{
		fprintf(fidH,"#define %s_id %d\n", gen->cuts->rod_list[i]->name, i);
	}
	fprintf(fidH,"\n\n");

	fprintf(fidH,"// solid\n");
	fprintf(fidH,"\n");
	for(i=0; i< gen->cuts->n_solid; i++)
	{
		fprintf(fidH,"#define %s_id %d\n", gen->cuts->solid_list[i]->name, i);
	}
	fprintf(fidH,"\n\n");

	fprintf(fidH,"// link\n");
	fprintf(fidH,"\n");
	for(i=0; i< gen->links->n_link; i++)
	{
		fprintf(fidH,"#define %s_id %d\n", gen->links->link_list[i]->name, i);
	}
	fprintf(fidH,"\n\n");

	fprintf(fidH,"// link3D\n");
	fprintf(fidH,"\n");
	for(i=0; i< gen->links->n_link3D; i++)
	{
		fprintf(fidH,"#define %s_id %d\n", gen->links->link3D_list[i]->name, i);
	}
	fprintf(fidH,"\n\n");
*/

	fprintf(fidH,"// sensor\n");
	fprintf(fidH,"\n");
	for(i=0; i< gen->n_sensor; i++)
	{
		fprintf(fidH,"#define %s_id %d\n", gen->point_list[gen->sensor_point_list[i]]->sensor->name, i);
	}
	fprintf(fidH,"\n\n");

	fprintf(fidH,"// extforce\n");
	fprintf(fidH,"\n");
	for(i=0; i< gen->n_extforce; i++)
	{
		fprintf(fidH,"#define %s_id %d\n", gen->point_list[gen->extforce_point_list[i]]->extforce->name, i);
	}
	fprintf(fidH,"\n\n");

	fprintf(fidH,"// ============================================================ //\n\n");
	fprintf(fidH,"# endif""\n");
	fclose(fidH);
	printf(">> 'user_all_id.h' created\n");

}