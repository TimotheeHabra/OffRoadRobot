//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "simu_def.h"

void user_Derivative(MBSdataStruct *MBSdata)
{
    UserIOStruct *uvs;

    uvs = MBSdata->user_IO;

    double  voltage, omega;

    double rho = MBSdata->user_IO->acs->GearRatio;
    double K_W = MBSdata->user_IO->acs->Kbemf;
    double L_M = MBSdata->user_IO->acs->Inductance;
    double R_M = MBSdata->user_IO->acs->Resistance;

    // PD control
    double Kp=200;
    double Kd=0.1;
//    double ref = 4* MBSdata->tsim;


    double *ref = MBSdata->user_IO->refs;


    // need a map from index i=0:4 to real joint indices
    // ux:current, uxd: current derivatives:

    // Front Right Motor ***********
        omega = rho* MBSdata->qd[R2_FR];
        //voltage = uvs->Voltage[M_FR];
        //voltage = 5; // overwrite the control signal
        // PD control law
        voltage = Kp*(ref[M_FR]-MBSdata->q[R2_FR])-Kd*MBSdata->qd[R2_FR];

    // Motor (electrical) ODE
        MBSdata->uxd[M_FR]= (1.0/L_M)*(voltage -R_M*MBSdata->ux[M_FR]-K_W*omega);

    // Front Left Motor ***********
        omega = rho* MBSdata->qd[R2_FL];
        //voltage = uvs->Voltage[M_FL];
        //voltage = 5; // overwrite the control signal
        // PD control law
        voltage = Kp*(ref[M_FL]-MBSdata->q[R2_FL])-Kd*MBSdata->qd[R2_FL];

        // Motor (electrical) ODE
        MBSdata->uxd[M_FL]= (1.0/L_M)*(voltage -R_M*MBSdata->ux[M_FL]-K_W*omega);

    // Rear Right Motor ***********
        omega = rho* MBSdata->qd[R2_RR];
        //voltage = uvs->Voltage[M_RR];
        //voltage = 5; // overwrite the control signal
        // PD control law
        voltage = Kp*(ref[M_RR]-MBSdata->q[R2_RR])-Kd*MBSdata->qd[R2_RR];
        // Motor (electrical) ODE
        MBSdata->uxd[M_RR]= (1.0/L_M)*(voltage - R_M*MBSdata->ux[M_RR]-K_W*omega);

    // Rear Left Motor ***********
        omega = rho* MBSdata->qd[R2_RL];
        //voltage = uvs->Voltage[M_RL];
        //voltage =5;  // overwrite the control signal
        // PD control law
        voltage = Kp*(ref[M_RL]-MBSdata->q[R2_RL])-Kd*MBSdata->qd[R2_RL];
        // Motor (electrical) ODE
        MBSdata->uxd[M_RL]= (1.0/L_M)*(voltage -R_M*MBSdata->ux[M_RL]-K_W*omega);

}

