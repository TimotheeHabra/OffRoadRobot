//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2008 
// Last update : 10/02/2008 
//--------------------------- 

#include "simu_def.h"

// Equivalent to user_DirDyn_io from Matlab
// Computes the position to give to the driven joints
#ifndef CMEX
void user_compute_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds)
#else
void user_compute_output(MBSdataStruct *MBSdata, LocalDataStruct *lds)
#endif
{
    simu_controller_loop(MBSdata);
}
