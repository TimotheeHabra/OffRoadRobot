//---------------------------
// Creation : ../../2013
// Last update : ../../2013
//---------------------------

#include "simu_def.h"

#ifdef YARP
	#include "yarp_interface.h"
#endif

void simu_controller_loop(MBSdataStruct *MBSdata)
{
    // controller variables
    ControllerStruct *cvs;

    #ifndef YARP
   		cvs = MBSdata->user_IO->cvs;

   		// controller
   		controller_inputs(MBSdata);
   		controller_loop(cvs);
   		controller_outputs(MBSdata);

   		// simulation outputs
   		simu_outputs(MBSdata);

   	#else //if def YARP

   		// TODO : ifdef YARP, then change argument of simu_controller_loop to remove MBSdata

   		getControllerInput_Yarp();
   		//controller_loop(cvs);  // uncomment when getControllerInput_Yarp will be working
   		writeControllerOutput_Yarp();
   	#endif
}
