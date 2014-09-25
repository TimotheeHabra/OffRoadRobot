/*
 * Initialization of the simulation
 *
 * authors: Nicolas Van der Noot and Allan Barrea
 */

/*
 * Initialization of the simulation
 *
 * authors: Nicolas Van der Noot and Allan Barrea
 */

#include "main_simulation.h"

#include "nrutil.h"
#include "real_time.h"
#include "cmake_config.h"

#include "mstr.h"
#include "user_all_id.h" // for the debug ; after delete it 

#ifdef UNIX
#include <time.h>
#include <sys/time.h>
#endif

#ifdef WIN32
#include <time.h>
#include <sys/timeb.h>
#endif

#define MBS_FILE PROJECT_ABS_PATH"/../dataR/"PROJECT_NAME_MBS

/*
 * Simulation initialization
 */
Loop_inputs* init_simulation()
{
	// -- Variables declaration -- //
	
	int i;
	int model_state_size;

    double *ystart;

	MSTR_strct * mstr =  NULL;

	MBSdataStruct *MBSdata   = NULL;
	LocalDataStruct *lds     = NULL;
	Loop_inputs *loop_inputs = NULL;

	PART_gen_strct*  part = NULL;

	#if defined(JNI) & defined (REAL_TIME)
	JNI_struct* jni_struct = NULL;
	#endif

	#ifdef WRITE_FILES
	Write_files *write_files;
	#endif

	#ifdef YARP
    void* RobotranYarp_interface = NULL;
    #endif

	struct timeval seed_time;

	// -- Seed for random -- //

	gettimeofday(&seed_time, NULL);
	srand(seed_time.tv_usec * seed_time.tv_sec);
    
    // -- Variables initialization -- //

    mstr= init_MSTR_strct();
	MSTR_exe_load(mstr, MBS_FILE); 

    //MSTR_exe_equil(mstr);

////////////////////////////////////////////////////////////////////

//    printf("u :  %d\n",mstr->MBSdata->nqu);
//	print_int_vec(&(mstr->MBSdata->qu[1]), mstr->MBSdata->nqu);
//	printf("\n");
//	printf("v :  %d\n",mstr->MBSdata->nqv);
//	print_int_vec(&(mstr->MBSdata->qv[1]), mstr->MBSdata->nqv);
//	printf("\n");
//	printf("hu :  %d\n",mstr->MBSdata->nhu);
//	print_int_vec(&(mstr->MBSdata->hu[1]), mstr->MBSdata->nhu);
//	printf("\n");

//    //MSTR_exe_part(mstr);

////    printf("u :  %d\n",mstr->part->n_qu);
////    print_int_vec(mstr->part->ind_qu, mstr->part->n_qu);
////    printf("\n");
////    printf("v :  %d\n",mstr->part->n_qv);
////    print_int_vec(mstr->part->ind_qv, mstr->part->n_qv);
////    printf("\n");
////    printf("hu :  %d\n",mstr->part->n_hu);
////    print_int_vec(mstr->part->ind_hu, mstr->part->n_hu);
////    printf("\n");
////    printf("hv :  %d\n",mstr->part->n_hv);
////    print_int_vec(mstr->part->ind_hv, mstr->part->n_hv);
////    printf("\n");

//    printf("u :  %d\n",mstr->MBSdata->nqu);
//    print_int_vec(&(mstr->MBSdata->qu[1]), mstr->MBSdata->nqu);
//    printf("\n");
//    printf("v :  %d\n",mstr->MBSdata->nqv);
//    print_int_vec(&(mstr->MBSdata->qv[1]), mstr->MBSdata->nqv);
//    printf("\n");
//    printf("hu :  %d\n",mstr->MBSdata->nhu);
//    print_int_vec(&(mstr->MBSdata->hu[1]), mstr->MBSdata->nhu);
//    printf("\n");

//    gen = mstr->mds;

//    sandbox = 1;
//        if(sandbox)
//        {

//            printf("gz :%f \n", gen->base->gravity[2]);

//            printf("n_body :%d \n", gen->bodytree->n_body);

//            for(i=0; i<gen->bodytree->n_body; i++)
//            {
//                printf("Name of body %d is : %s \n", i,gen->bodytree->body_list[i]->name);
//            }
//            for(i=0; i<gen->bodytree->n_body; i++)
//            {
//                printf("Name parent of body %d is : %s \n", i,gen->bodytree->body_list[i]->parent->bodyname);
//            }
//            for(i=0; i<gen->bodytree->n_body; i++)
//            {
//                printf("Mass of body %d is : %f kg\n", i,gen->bodytree->body_list[i]->mass);
//            }
//            for(i=0; i<gen->bodytree->n_body; i++)
//            {
//                printf("Center of mass of body %d is : %f, %f, %f\n", i,gen->bodytree->body_list[i]->com[0], gen->bodytree->body_list[i]->com[1] ,gen->bodytree->body_list[i]->com[2]);
//            }
//            for(i=0; i<gen->bodytree->n_body; i++)
//            {
//                printf("Inertia matrix of body %d is : %f, %f, %f, %f, %f, %f\n", i,gen->bodytree->body_list[i]->inertia[0],gen->bodytree->body_list[i]->inertia[1],gen->bodytree->body_list[i]->inertia[2],gen->bodytree->body_list[i]->inertia[3],gen->bodytree->body_list[i]->inertia[4],gen->bodytree->body_list[i]->inertia[5] );
//            }
//            for(i=0; i<gen->bodytree->n_body; i++)
//            {
//                printf("Number of point of body %d is : %d\n", i,gen->bodytree->body_list[i]->n_point );
//            }
//            for(i=0; i<gen->bodytree->n_body; i++)
//            {
//                printf("Point of body %d is :\n", i);
//                for(j=0; j<gen->bodytree->body_list[i]->n_point; j++)
//                {
//                    printf("Point %d is : %f, %f, %f \n", j, gen->bodytree->body_list[i]->point_list[j]->pt[0], gen->bodytree->body_list[i]->point_list[j]->pt[1], gen->bodytree->body_list[i]->point_list[j]->pt[2]);
//                    printf("%s\n", gen->bodytree->body_list[i]->point_list[j]->name);
//                }
//            }
//            for(i=0; i<gen->bodytree->n_body; i++)
//            {
//                printf("Joint of body %d is :\n", i);
//                for(j=0; j<gen->bodytree->body_list[i]->n_joint; j++)
//                {
//                    printf("Joint %d is %s : type %d, nature %d \n", j,  gen->bodytree->body_list[i]->joint_list[j]->name, gen->bodytree->body_list[i]->joint_list[j]->type, gen->bodytree->body_list[i]->joint_list[j]->nature);
//                }
//            }
//            printf("It is the cuts \n");
//            for(i=0; i<gen->cuts->n_ball; i++)
//            {
//                printf("Ball %d is %s based on point: %s in %s and on point: %s in %s \n", i, gen->cuts->ball_list[i]->name, gen->cuts->ball_list[i]->endpoint1->pointname, gen->cuts->ball_list[i]->endpoint1->bodyname,gen->cuts->ball_list[i]->endpoint2->pointname, gen->cuts->ball_list[i]->endpoint2->bodyname);
//            }
//            printf("It is the links \n");
//            for(i=0; i<gen->links->n_link; i++)
//            {
//                printf("Link %d is %s based on point: %s in %s and on point: %s in %s \n", i, gen->links->link_list[i]->name, gen->links->link_list[i]->endpoint1->pointname, gen->links->link_list[i]->endpoint1->bodyname,gen->links->link_list[i]->endpoint2->pointname, gen->links->link_list[i]->endpoint2->bodyname);
//            }

//            printf("qu : %d -> [", gen->bodytree->n_qu);
//            for(i=0; i<gen->bodytree->n_qu; i++)
//            {
//                printf(" %d",  gen->bodytree->qu[i]) ;
//            }
//            printf(" ]\n");

//            printf("qv : %d -> [", gen->bodytree->n_qv);
//            for(i=0; i<gen->bodytree->n_qv; i++)
//            {
//                printf(" %d",  gen->bodytree->qv[i]) ;
//            }
//            printf(" ]\n");

//            printf("qc : %d -> [", gen->bodytree->n_qc);
//            for(i=0; i<gen->bodytree->n_qc; i++)
//            {
//                printf(" %d",  gen->bodytree->qc[i]) ;
//            }
//            printf(" ]\n");

//            printf("qa : %d -> [", gen->bodytree->n_qa);
//            for(i=0; i<gen->bodytree->n_qc; i++)
//            {
//                printf(" %d",  gen->bodytree->qa[i]) ;
//            }
//            printf(" ]\n");

//            gen->bodytree->joint_list[1]->nature = 5;

//            for(i=0; i<gen->bodytree->n_body; i++)
//            {
//                printf("Joint of body %d is :\n", i);
//                for(j=0; j<gen->bodytree->body_list[i]->n_joint; j++)
//                {
//                    printf("Joint %d is %s : type %d, nature %d \n", j,  gen->bodytree->body_list[i]->joint_list[j]->name, gen->bodytree->body_list[i]->joint_list[j]->type, gen->bodytree->body_list[i]->joint_list[j]->nature);
//                }
//            }

//            printf("n_point : %d \n", gen->n_point);

//            printf("All points: \n");
//            for(i=0; i<gen->n_point; i++)
//            {
//                printf("point %d is %f , %f, %f\n",i , gen->point_list[i]->pt[0], gen->point_list[i]->pt[1], gen->point_list[i]->pt[2]);
//            }

//            gen->point_list[1]->pt[0] = 3.14;

//            printf("%f", gen->bodytree->body_list[0]->point_list[1]->pt[0] );

//            printf("n_sensor : %d \n", gen->n_sensor);
//            printf("n_extforce : %d \n", gen->n_extforce);

//        //	printf("Compare 2 : %d vs %d",MBSdata->qu[13], MBSdata2->qu[13]);

//            printf("END TEST \n");


//            //system("pause");
//            i = 0;
//            j = 0;


//            printf("usermodels:\n");
//            for(i=0; i<gen->user_models->n_user_model; i++)
//            {
//                printf("%s:\n",gen->user_models->user_model_list[i]->name);
//                for(j=0; j<gen->user_models->user_model_list[i]->n_parameter; j++)
//                {
//                    printf("	%s:\n",gen->user_models->user_model_list[i]->parameter_list[j]->name);
//                    printf("		Type: %d \n		Value: ",gen->user_models->user_model_list[i]->parameter_list[j]->type);
//                    for(k=0; k<gen->user_models->user_model_list[i]->parameter_list[j]->n_value; k++)
//                    {
//                        printf("%f ",gen->user_models->user_model_list[i]->parameter_list[j]->value_list[k]);
//                    }
//                    printf("\n");
//                }
//            }


//            //printf("pt : %d\n", MBSdata->npt);
//            //print_double_tab(&(MBSdata->dpt[1]), 3, MBSdata->npt+1);


//            //system("pause");

//        }
//        // end of sand box

//        for(j=1; j<=14; j++)
//        {
//            for(i=0; i<=14; i++)
//            {
//                //printf("Mr[%d][%d] = %f \n",j,i,lds->Mr[j][i]);
//                printf("mstr Mr[%d][%d] = %f \n",j,i, mstr->lds->M[j][i]);
//            }
//        }

//    printf("pause, press a key to go on \n");
//    getchar();


	MBSdata = mstr->MBSdata; 
	lds = mstr->lds;

	if(MBSdata == NULL)
	{
        printf("error while loading MBSdata \n");
	}
	#ifdef PRINT_REPORT
	else
	{
        printf("MBSdata successfully loaded \n");
	}
	#endif

    // LocalDataStruct initialization

    #if !defined ACCELRED
    lds = initLocalDataStruct(MBSdata);
    #else
    lds = NULL;
    #endif

	if(lds == NULL)
	{
	    printf("error while initializing LocalDataStruct\n");
	}
	#ifdef PRINT_REPORT
	else
	{
	    printf("LocalDataStruct successfully initialized\n");
	}
	#endif

	// Yarp Initialization
	#ifdef YARP
    RobotranYarp_interface = yarp_init();
    if(RobotranYarp_interface == NULL)
    {
        printf("******************\nSomething went wrong during YARP initialization... what to do here?\n******************\n");
    }
    #endif

	// Model initialization
	if(user_initialization(MBSdata, lds))
	{
	    printf("error in user_initialization\n");
	}
	#ifdef PRINT_REPORT
	else
	{
	    printf("Model successfully initialized\n");
	}
	#endif

	// Integrator parameters initialization
	model_state_size = 2*MBSdata->nqu + MBSdata->Nux;

	#ifdef PRINT_REPORT
	printf("model_state_size: %d\n", model_state_size);
	#endif

	ystart = dvector(1,model_state_size);

	#ifdef WRITE_FILES
	write_files = init_write_files(NB_SIMU_STEPS, MBSdata->njoint);
	#endif

    // Simulation state initialization
	for(i=1; i<=MBSdata->nqu; i++)
	{
		ystart[i]              = MBSdata->q[MBSdata->qu[i]];
		ystart[i+MBSdata->nqu] = MBSdata->qd[MBSdata->qu[i]];
	}

	for(i=1; i<=MBSdata->Nux; i++)
	{
	    ystart[i+2*MBSdata->nqu] = MBSdata->ux[i];
	}

	// JNI visualization
	#if defined(JNI) & defined (REAL_TIME)
	jni_struct = init_jni(MBSdata);
	#endif

    // initialize the inputs of the simulation loop
    loop_inputs = (Loop_inputs*) malloc(sizeof(Loop_inputs));

    // absolute initial time of the simulation
    time_get(&(loop_inputs->init_t_sec), &(loop_inputs->init_t_usec));

    loop_inputs->nvar    = model_state_size;
    loop_inputs->nstep   = NB_SIMU_STEPS;
    loop_inputs->x1      = TSIM_INIT;
    loop_inputs->x2      = TSIM_END;
    loop_inputs->ystart  = ystart;
    loop_inputs->lds     = lds;
    loop_inputs->MBSdata = MBSdata;

    #ifdef WRITE_FILES
    loop_inputs->write_files = write_files;
    #endif

    // Real-time constraiints
    #ifdef REAL_TIME
    loop_inputs->real_time = init_real_time(loop_inputs->init_t_sec, loop_inputs->init_t_usec);
    #endif

    // SDL window
    #if defined(SDL) & defined (REAL_TIME)	
	loop_inputs->screen_sdl = configure_screen_sdl(loop_inputs->init_t_sec, loop_inputs->init_t_usec);
	#endif

	#if defined(JNI) & defined (REAL_TIME)
    loop_inputs->jni_struct = jni_struct;
    update_jni(loop_inputs->jni_struct, MBSdata, loop_inputs->real_time);
    #endif

    #ifdef YARP
    loop_inputs->RobotranYarp_interface = RobotranYarp_interface;
    #endif

    // Running model integration
    #ifdef PRINT_REPORT
    printf("Running integration of the model...\n");
    #endif

    return loop_inputs;
} 

