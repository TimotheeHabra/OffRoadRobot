//---------------------------
// Creation : ../../2013
// Last update : ../../2013
//---------------------------

#include "simu_def.h"

void controller_outputs(MBSdataStruct *MBSdata)
{
	int id = 0;
	UserIOStruct *uvs;
    uvs = MBSdata->user_IO;

	// Front right leg
	id = R2_FR;

	MBSdata->q[id]    	= uvs->cvs->Outputs->q_ref[0];
	MBSdata->qd[id]     = uvs->cvs->Outputs->qd_ref[0];
	MBSdata->qdd[id]    = uvs->cvs->Outputs->qdd_ref[0];

	// Front left leg
	id = R2_FL;

	MBSdata->q[id]    	= uvs->cvs->Outputs->q_ref[1];
	MBSdata->qd[id]     = uvs->cvs->Outputs->qd_ref[1];
	MBSdata->qdd[id]    = uvs->cvs->Outputs->qdd_ref[1];

    //Rear Right Leg
    id = R2_RR;

	MBSdata->q[id]    	= uvs->cvs->Outputs->q_ref[2];
	MBSdata->qd[id]     = uvs->cvs->Outputs->qd_ref[2];
	MBSdata->qdd[id]    = uvs->cvs->Outputs->qdd_ref[2];

    //Rear Left Leg
    id = R2_RL;

	MBSdata->q[id]    	= uvs->cvs->Outputs->q_ref[3];
	MBSdata->qd[id]     = uvs->cvs->Outputs->qd_ref[3];
	MBSdata->qdd[id]    = uvs->cvs->Outputs->qdd_ref[3];

    // Sending voltage to the motors:
    uvs->Voltage[M_FR]=10.0;
    uvs->Voltage[M_FL]=10.0;
    uvs->Voltage[M_RR]=10.0;
    uvs->Voltage[M_RL]=10.0;

}

