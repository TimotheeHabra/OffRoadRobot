//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "simu_def.h"

double* user_JointForces(MBSdataStruct *MBSdata, double tsim)
{

	double K = 10.0;

    double rho = MBSdata->user_IO->acs->GearRatio;
    double K_T = MBSdata->user_IO->acs->TrqConst;

	MBSdata->Qq[Spring_FR] = -K * MBSdata->q[Spring_FR];
	MBSdata->Qq[Spring_FL] = -K * MBSdata->q[Spring_FL];
	MBSdata->Qq[Spring_RR] = -K * MBSdata->q[Spring_RR];
	MBSdata->Qq[Spring_RL] = -K * MBSdata->q[Spring_RL];

    // Actuated Joints:

    MBSdata->Qq[R2_FR] = rho* K_T * MBSdata->ux[M_FR];
    MBSdata->Qq[R2_FL] = rho* K_T * MBSdata->ux[M_FL];
    MBSdata->Qq[R2_RR] = rho* K_T * MBSdata->ux[M_RR];
    MBSdata->Qq[R2_RL] = rho* K_T * MBSdata->ux[M_RL];

   	return MBSdata->Qq;


}

