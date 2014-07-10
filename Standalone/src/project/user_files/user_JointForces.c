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

	double K = 50.0;
    double rho = MBSdata->user_IO->actuatorsStruct->acs[0]->GearRatio;
    double KT = MBSdata->user_IO->actuatorsStruct->acs[0]->Kbemf;
    // Transmission stiffness, damping
    double Ks = MBSdata->user_IO->actuatorsStruct->acs[0]->SeriesSpring;
    double Ds = MBSdata->user_IO->actuatorsStruct->acs[0]->SeriesDamping;

	MBSdata->Qq[Spring_FR] = -K * MBSdata->q[Spring_FR];
	MBSdata->Qq[Spring_FL] = -K * MBSdata->q[Spring_FL];
	MBSdata->Qq[Spring_RR] = -K * MBSdata->q[Spring_RR];
	MBSdata->Qq[Spring_RL] = -K * MBSdata->q[Spring_RL];


    // Actuated Joints:
    if (Act_type==1) //SEA
    {

        switch (Act_order) {
          case 1:
            justElectrical:
            MBSdata->Qq[R2_FR] = rho* KT * MBSdata->ux[M_FR];
            MBSdata->Qq[R2_FL] = rho* KT * MBSdata->ux[M_FL];
            MBSdata->Qq[R2_RR] = rho* KT * MBSdata->ux[M_RR];
            MBSdata->Qq[R2_RL] = rho* KT * MBSdata->ux[M_RL];
          break;

          case 2:
            justMechanical:
            MBSdata->Qq[R2_FR]=Ks*(MBSdata->ux[M_FR]-MBSdata->q[R2_FR])+Ds*(MBSdata->uxd[M_FR]-MBSdata->qd[R2_FR]);
            MBSdata->Qq[R2_FL]=Ks*(MBSdata->ux[M_FL]-MBSdata->q[R2_FL])+Ds*(MBSdata->uxd[M_FL]-MBSdata->qd[R2_FL]);
            MBSdata->Qq[R2_RR]=Ks*(MBSdata->ux[M_RR]-MBSdata->q[R2_RR])+Ds*(MBSdata->uxd[M_RR]-MBSdata->qd[R2_RR]);
            MBSdata->Qq[R2_RL]=Ks*(MBSdata->ux[M_RL]-MBSdata->q[R2_RL])+Ds*(MBSdata->uxd[M_RL]-MBSdata->qd[R2_RL]);
          break;

          case 3:
            goto justMechanical;
          break;

          default:
            printf("detault actuator order (1) selected \n");
            goto justElectrical;
          break;
        }
    }
   	return MBSdata->Qq;


}

