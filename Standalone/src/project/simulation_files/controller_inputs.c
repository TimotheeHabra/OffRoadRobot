//---------------------------
// Creation : ../../2013
// Last update : ../../2013
//---------------------------

#include "simu_def.h"

void controller_inputs(MBSdataStruct *MBSdata)
{
    UserIOStruct *uvs;
    ControllerStruct *cvs;
    ControllerInputsStruct *ivs;

    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;
    ivs = cvs->Inputs;
    
    // ---- Simulation inputs ---- //
    
    // ---- Controller inputs ---- //
    ivs->tsim = MBSdata->tsim; // time [s]
  
}

