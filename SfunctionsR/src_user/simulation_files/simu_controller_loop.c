//---------------------------
// Creation : ../../2013
// Last update : ../../2013
//---------------------------

#include "simu_def.h"

void simu_controller_loop(MBSdataStruct *MBSdata)
{
    // controller variables
    ControllerStruct *cvs;
    cvs = MBSdata->user_IO->cvs;
    
    // controller
    controller_inputs(MBSdata);
    controller_loop(cvs);
    controller_outputs(MBSdata);

    // simulation outputs
    simu_outputs(MBSdata);
}
