/*
 * Header of the main files
 *
 * author: Nicolas Van der Noot
 */

#ifndef __MAIN_SIMULATION_H_INCLUDED__  // guard against multiple/recursive includes
#define __MAIN_SIMULATION_H_INCLUDED__

#include "MBSdataStruct.h"
#include "project_info.h"

#if defined(JNI) & defined (REAL_TIME)
#include "jni_functions.h"
#endif

#ifdef WRITE_FILES 
#include "write_files.h"
#endif

#if defined(SDL) & defined (REAL_TIME)
#include "plot_sdl.h"
#endif

#ifdef REAL_TIME
#include "real_time.h"
#endif

#ifdef YARP
#include "yarp_files.h"
#endif

// -- Macros -- //

#define NB_SIMU_STEPS ( (int) ((TSIM_END - TSIM_INIT) / DELTA_TSIM) )

#define PATH_MAX_LENGTH 200

// inputs of the main loop
typedef struct Loop_inputs
{
	int init_t_sec; 
    int init_t_usec;
    int nvar;
    int nstep;

    double x1;
    double x2; 

    double *ystart;
    
    LocalDataStruct *lds; 
    MBSdataStruct *MBSdata; 

    #ifdef WRITE_FILES
    Write_files *write_files;
    #endif

    #if defined(JNI) & defined (REAL_TIME)
    JNI_struct* jni_struct;
    #endif

    #ifdef REAL_TIME
    Simu_real_time *real_time;
    #endif

    #if defined(SDL) & defined (REAL_TIME)
    Screen_sdl *screen_sdl;
    #endif

    #ifdef YARP
    void* RobotranYarp_interface;
    #endif
    
} Loop_inputs;

// -- Prototypes -- //

void main_simulation();								// main
Loop_inputs* init_simulation();					// initialization
void loop_simulation(Loop_inputs *loop_inputs);		// loop
void finish_simulation(Loop_inputs *loop_inputs);	// end of the simulation

#endif
