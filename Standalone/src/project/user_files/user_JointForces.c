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

	MBSdata->Qq[Spring_FR] = -K * MBSdata->q[Spring_FR];
	MBSdata->Qq[Spring_FL] = -K * MBSdata->q[Spring_FL];
	MBSdata->Qq[Spring_RR] = -K * MBSdata->q[Spring_RR];
	MBSdata->Qq[Spring_RL] = -K * MBSdata->q[Spring_RL];

   	return MBSdata->Qq;
} 

