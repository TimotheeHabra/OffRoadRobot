/*
 * End of the simulation (print reports, free memory...)
 * 
 * author: Nicolas Van der Noot
 */

#include "main_simulation.h"

#include "loadMBSheader_xml.h"
#include "real_time.h"
#include "cmake_config.h"

/*
 * End of the simulation
 */
void finish_simulation(Loop_inputs *loop_inputs)
{
    // -- Variables declaration -- //

    #ifdef PRINT_REPORT
	double total_t_sec;
    #endif

    MBSdataStruct *MBSdata = NULL;

    // -- Simulation end -- //

    // for the anim file
    #ifdef WRITE_FILES
	#ifdef REAL_TIME
	const char *fileout_anim = PROJECT_ABS_PATH"/../animationR/Model_real_time.anim";
	#else
	const char *fileout_anim = PROJECT_ABS_PATH"/../animationR/Model_standalone.anim";
	#endif
    const char generic_fileout[PATH_MAX_LENGTH] = PROJECT_ABS_PATH"/src/other/outVectors/output_vector";
	#endif

	// MBSdata
	MBSdata = loop_inputs->MBSdata;

    
    #ifdef PRINT_REPORT
    // total real-time needed for the simulation
    total_t_sec = t_usec(loop_inputs->init_t_sec, loop_inputs->init_t_usec) / 1.0e6;

    // Printing simulation report
    printf("... Done\n\n");
    printf("-------------------------------\n");
    printf("nb steps:\t%d\n", NB_SIMU_STEPS);
    printf("time step:\t%.3f\t[ms]\n", DELTA_TSIM * 1000);
    printf("-------------------------------\n");
    printf("t_start:\t%.3f\t[s]\n", TSIM_INIT);
    printf("t_end:\t\t%.3f\t[s]\n", TSIM_END);
    printf("-------------------------------\n");
    printf("Final time:\t%.3f\t[s]\n", MBSdata->tsim);
	printf("Executed in:\t%.3f\t[s]\n", total_t_sec);
    printf("-------------------------------\n\n");
	#endif

    // writing the .anim file
    #ifdef WRITE_FILES

    loop_inputs->write_files->kount = loop_inputs->write_files->kount - 1; //last increment doesn't correspond to a writting

    if(write_anim_file(loop_inputs->write_files, MBSdata->njoint, fileout_anim))
    {
        printf("error: cannot write the anim file\n");
    }
    #ifdef PRINT_REPORT
    else
    {
        printf("File '%s' successfully written\n", fileout_anim);
    }
    #endif

    if (write_out_files(loop_inputs->write_files, generic_fileout))
    {
        printf("error: cannot write the output vectors files\n");
    }
    #ifdef PRINT_REPORT
    else
    {
        printf("Output Files '%s' successfully written\n", generic_fileout);
    }
    #endif

    #endif

	// -- Closing operations -- //

    // .anim files
	#ifdef WRITE_FILES
	free_write_files(loop_inputs->write_files, MBSdata->njoint);
	#endif

    // JNI visualization
    #if defined(JNI) & defined (REAL_TIME)
	free_jni(loop_inputs->jni_struct);
    #endif

    // Real-time constraints
    #ifdef REAL_TIME
	free_simu_real_time(loop_inputs->real_time);
    #endif

    // SDL window
    #if defined(SDL) & defined(REAL_TIME)
	free_screen_sdl(loop_inputs->screen_sdl);
    #endif

    // LocalDataStruct
    #if !defined ACCELRED
    freeLocalDataStruct(loop_inputs->lds,MBSdata);
    #endif

    #ifdef YARP
    yarp_finish(loop_inputs->RobotranYarp_interface);
    #endif

    // MBSdata_xml
    freeMBSdata_xml(MBSdata);
    
    // simulation loop inputs
	free(loop_inputs);
}
