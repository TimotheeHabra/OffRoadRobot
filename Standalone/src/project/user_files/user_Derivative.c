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

    int i;

    double rho ;
    double K_W ;
    double L_M ;
    double R_M ;
    double KT  ;

    double Ks ;
    double Ds ;
    double J_M;
    // voltage to torque gain (used in 2nd order dynamics)
    double VT ; //
    double D_M;

    const int n = NB_ACTUATED_JOINTS;

    // PD control !!! the gains should change depending on the actuator order and servo_type
    double Kp=100;
    double Kd=0.1;

    double voltage[NB_ACTUATED_JOINTS]={0.0};
    double Cpl[NB_ACTUATED_JOINTS]={0.0};

    double *ref = MBSdata->user_IO->refs;


    uvs = MBSdata->user_IO;

    if (Act_type==1) //SEA
    {
        // PD control law
        voltage[M_FR] = Kp*(ref[M_FR]-MBSdata->q[R2_FR])-Kd*MBSdata->qd[R2_FR];
        voltage[M_FL] = Kp*(ref[M_FL]-MBSdata->q[R2_FL])-Kd*MBSdata->qd[R2_FL];
        voltage[M_RR] = Kp*(ref[M_RR]-MBSdata->q[R2_RR])-Kd*MBSdata->qd[R2_RR];
        voltage[M_RL] = Kp*(ref[M_RL]-MBSdata->q[R2_RL])-Kd*MBSdata->qd[R2_RL];

        switch (Act_order) {
          case 1:
            justElectrical:
            // Motor (electrical) ODE
            // need a map from index i=0:4 to real joint indices
            // ux:current, uxd: current derivatives:
            rho = MBSdata->user_IO->acs[M_FR]->GearRatio;
            R_M = MBSdata->user_IO->acs[M_FR]->Resistance;
            K_W = MBSdata->user_IO->acs[M_FR]->Kbemf;
            L_M = MBSdata->user_IO->acs[M_FR]->Inductance;

            // Front Right Motor ***********
            MBSdata->uxd[M_FR]= (1.0/L_M)*(voltage[M_FR] -R_M*MBSdata->ux[M_FR]-K_W*rho* MBSdata->qd[R2_FR]);
// THE FOLLOWING LINES WILL BE REPLACED LATER WITH A MAP ITERATION
//            rho = MBSdata->user_IO->acs[M_FL]->GearRatio;
//            R_M = MBSdata->user_IO->acs[M_FL]->Resistance;
//            K_W = MBSdata->user_IO->acs[M_FL]->Kbemf;
//            L_M = MBSdata->user_IO->acs[M_FL]->Inductance;
            // Front Left Motor ***********
            MBSdata->uxd[M_FL]= (1.0/L_M)*(voltage[M_FL] -R_M*MBSdata->ux[M_FL]-K_W*rho* MBSdata->qd[R2_FL]);

//            rho = MBSdata->user_IO->acs[M_RR]->GearRatio;
//            R_M = MBSdata->user_IO->acs[M_RR]->Resistance;
//            K_W = MBSdata->user_IO->acs[M_RR]->Kbemf;
//            L_M = MBSdata->user_IO->acs[M_RR]->Inductance;
            // Rear Right Motor ***********
            MBSdata->uxd[M_RR]= (1.0/L_M)*(voltage[M_RR] - R_M*MBSdata->ux[M_RR]-K_W*rho* MBSdata->qd[R2_RR]);

//            rho = MBSdata->user_IO->acs[M_RL]->GearRatio;
//            R_M = MBSdata->user_IO->acs[M_RL]->Resistance;
//            K_W = MBSdata->user_IO->acs[M_RL]->Kbemf;
//            L_M = MBSdata->user_IO->acs[M_RL]->Inductance;
            // Rear Left Motor ***********
            MBSdata->uxd[M_RL]= (1.0/L_M)*(voltage[M_RL] -R_M*MBSdata->ux[M_RL]-K_W*rho* MBSdata->qd[R2_RL]);
           break;
            case 2:
            // Motor (Mechanical) ODE
            // ux:motor position, velocity, uxd: motor velocity, acceleration
            //update motor velocities:
            for (i=0; i<n; i++)
            {
                MBSdata->uxd[i]=MBSdata->ux[i+n];
            }

            J_M = MBSdata->user_IO->acs[M_RR]->Inertia;
            VT  = rho*(KT)/R_M;
            D_M = MBSdata->user_IO->acs[M_RR]->Damping;
            Ks  = MBSdata->user_IO->acs[M_RR]->SeriesSpring;
            Ds  = MBSdata->user_IO->acs[M_RR]->SeriesDamping;

            // computing the transmission torque (coupling between motor and load)
            Cpl[M_FR]=Ks*(MBSdata->ux[M_FR]-MBSdata->q[R2_FR])+Ds*(MBSdata->uxd[M_FR]-MBSdata->qd[R2_FR]);
            Cpl[M_FL]=Ks*(MBSdata->ux[M_FL]-MBSdata->q[R2_FL])+Ds*(MBSdata->uxd[M_FL]-MBSdata->qd[R2_FL]);
            Cpl[M_RR]=Ks*(MBSdata->ux[M_RR]-MBSdata->q[R2_RR])+Ds*(MBSdata->uxd[M_RR]-MBSdata->qd[R2_RR]);
            Cpl[M_RL]=Ks*(MBSdata->ux[M_RL]-MBSdata->q[R2_RL])+Ds*(MBSdata->uxd[M_RL]-MBSdata->qd[R2_RL]);

            //update motor accelerations:
            MBSdata->uxd[M_FR]= (1.0/J_M)*(VT*voltage[M_FR] -D_M*MBSdata->ux[n+M_FR]-Cpl[M_FR]);
            MBSdata->uxd[M_FL]= (1.0/J_M)*(VT*voltage[M_FL] -D_M*MBSdata->ux[n+M_FL]-Cpl[M_FL]);
            MBSdata->uxd[M_RR]= (1.0/J_M)*(VT*voltage[M_RR] -D_M*MBSdata->ux[n+M_RR]-Cpl[M_RR]);
            MBSdata->uxd[M_RL]= (1.0/J_M)*(VT*voltage[M_RL] -D_M*MBSdata->ux[n+M_RL]-Cpl[M_RL]);
           break;
            case 3:
            // Motor (Electrical+Mechanical) ODE
            // ux:motor position, velocity, current uxd: motor velocity, acceleration, current derivative
            //update motor velocities:
            for (i=0; i<n; i++)
            {
                MBSdata->uxd[i]=MBSdata->ux[i+n];
            }

            rho = MBSdata->user_IO->acs[M_FR]->GearRatio;
            R_M = MBSdata->user_IO->acs[M_FR]->Resistance;
            K_W = MBSdata->user_IO->acs[M_FR]->Kbemf;
            L_M = MBSdata->user_IO->acs[M_FR]->Inductance;
            KT = K_W;

            J_M = MBSdata->user_IO->acs[M_RR]->Inertia;
            VT  = rho*(KT)/R_M;
            D_M = MBSdata->user_IO->acs[M_RR]->Damping;
            Ks  = MBSdata->user_IO->acs[M_RR]->SeriesSpring;
            Ds  = MBSdata->user_IO->acs[M_RR]->SeriesDamping;

            // computing the transmission torque (coupling between motor and load)
            Cpl[M_FR]=Ks*(MBSdata->ux[M_FR]-MBSdata->q[R2_FR])+Ds*(MBSdata->uxd[M_FR]-MBSdata->qd[R2_FR]);
            Cpl[M_FL]=Ks*(MBSdata->ux[M_FL]-MBSdata->q[R2_FL])+Ds*(MBSdata->uxd[M_FL]-MBSdata->qd[R2_FL]);
            Cpl[M_RR]=Ks*(MBSdata->ux[M_RR]-MBSdata->q[R2_RR])+Ds*(MBSdata->uxd[M_RR]-MBSdata->qd[R2_RR]);
            Cpl[M_RL]=Ks*(MBSdata->ux[M_RL]-MBSdata->q[R2_RL])+Ds*(MBSdata->uxd[M_RL]-MBSdata->qd[R2_RL]);
            // update motor acceleration:
            MBSdata->uxd[M_FR+n]= (1.0/J_M)*(KT*MBSdata->ux[2*n+R2_FR] -D_M*MBSdata->ux[n+M_FR]-Cpl[M_FR]);
            MBSdata->uxd[M_FL+n]= (1.0/J_M)*(KT*MBSdata->ux[2*n+R2_FL] -D_M*MBSdata->ux[n+M_FL]-Cpl[M_FL]);
            MBSdata->uxd[M_RR+n]= (1.0/J_M)*(KT*MBSdata->ux[2*n+R2_RR] -D_M*MBSdata->ux[n+M_RR]-Cpl[M_RR]);
            MBSdata->uxd[M_RL+n]= (1.0/J_M)*(KT*MBSdata->ux[2*n+R2_RL] -D_M*MBSdata->ux[n+M_RL]-Cpl[M_RL]);

            // update current derivative:
            MBSdata->uxd[M_FR+2*n]=(1.0/L_M)*(voltage[M_FR] -R_M*MBSdata->ux[2*n+M_FR]-K_W*rho* MBSdata->qd[R2_FR]);
            MBSdata->uxd[M_FL+2*n]=(1.0/L_M)*(voltage[M_FL] -R_M*MBSdata->ux[2*n+M_FL]-K_W*rho* MBSdata->qd[R2_FL]);
            MBSdata->uxd[M_RR+2*n]=(1.0/L_M)*(voltage[M_RR] -R_M*MBSdata->ux[2*n+M_RR]-K_W*rho* MBSdata->qd[R2_RR]);
            MBSdata->uxd[M_RL+2*n]=(1.0/L_M)*(voltage[M_RL] -R_M*MBSdata->ux[2*n+M_RL]-K_W*rho* MBSdata->qd[R2_RL]);
           break;
            default:
            printf("detault actuator order (1) selected \n");
            goto justElectrical;
           break;
            }
    }


}

