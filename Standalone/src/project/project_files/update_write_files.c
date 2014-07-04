/*
 * Header of the output vetors configuration
 *
 * author: Nicolas Van der Noot
 */

#ifdef WRITE_FILES

#include "write_files.h"
#include "main_simulation.h"
#include "useful_functions.h"

#include "controller_def.h"

/*
 * Update the 'Write_files' structure'
 */
void update_write_files(Write_files *write_files, MBSdataStruct *MBSdata)
{
    int i;
    int kount;
    double *t;
    double **qq, **out_vec;

    //UserIOStruct *uvs;
    //ControllerStruct *cvs;

    t       = write_files->t;
    qq      = write_files->qq;
    kount   = write_files->kount;
    out_vec = write_files->out_vec;

    //uvs = MBSdata->user_IO;
    //cvs = uvs->cvs;

    t[kount] = MBSdata->tsim;

    for(i=0; i<MBSdata->njoint; i++)
    {
        qq[i][kount] = MBSdata->q[i+1];
    }

    // -- TO MODIFY -- //
	/*
	 * You can assign values for the debugging vectors located in Standalone/other/outVectors
	 * 
	 * example:
	 *    out_vec[0][kount] = MBSdata->q[1];
	 *
	 * The fisrt index must be in the [0 ; 4] interval (0 corresponding to output_vec_1.txt)
	 * The second index is always 'kount'
	 */

    
    for(i=0; i<NB_OUTPUT_VEC; i++)
    {
        out_vec[i][kount] = 0.0;
    }

    // --------------- //

    kount++;
    write_files->kount = kount;
}

#endif

