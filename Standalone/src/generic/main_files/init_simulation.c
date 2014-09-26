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

#include "loadMBSdata_xml.h"
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

	MBSdataStruct *MBSdata   = NULL;
	LocalDataStruct *lds     = NULL;
	Loop_inputs *loop_inputs = NULL;

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

    MBSdata = loadMBSdata_xml(MBS_FILE);

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

