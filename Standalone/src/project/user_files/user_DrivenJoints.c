//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "simu_def.h"
//#include <math.h>

#define GO

// Sets the pos, vel, acc of driven joints (if different from 0)
void user_DrivenJoints(MBSdataStruct *MBSdata, double tsim)
{
    UserIOStruct *uvs;
    double T = 1.0; //[s]
	double My_PI = 3.1415926;
	double omega = 2*My_PI/(2*T); //[rad/s]
	int id = 0;

	uvs = MBSdata->user_IO;


#ifdef GO
	//Front Right Leg
//     id = R2_FR;
// 	if(fmod(tsim,2*T) < T)
// 	{
//         MBSdata->q[id]    	= My_PI*(-cos(omega*tsim)+1+floor(tsim));
//         MBSdata->qd[id]      = My_PI*omega*sin(omega*tsim);
//         MBSdata->qdd[id]     = My_PI*omega*omega*cos(omega*tsim);
//     }
//     else
//     {
//         MBSdata->q[id]   = My_PI*(1+floor(tsim));
//         MBSdata->qd[id]  = 0;
//         MBSdata->qdd[id] = 0;
//     }

    //Front Left Leg
 //    id = R2_FL;
 //    if(fmod(tsim,2*T) >= T)
	// {
 //        MBSdata->q[id]    	= My_PI*(-cos(omega*tsim + My_PI)+floor(tsim));
 //        MBSdata->qd[id]      = My_PI*omega*sin(omega*tsim + My_PI);
 //        MBSdata->qdd[id]     = My_PI*omega*omega*cos(omega*tsim + My_PI);
 //    }
 //    else
 //    {
 //        MBSdata->q[id]   = My_PI*(floor(tsim));
 //        MBSdata->qd[id]  = 0;
 //        MBSdata->qdd[id] = 0;
 //    }

    //Rear Right Leg
 //    id = R2_RR;
 //    if(fmod(tsim,2*T) >= T)
	// {
 //        MBSdata->q[id]    	 = My_PI*(-cos(omega*tsim + My_PI)+floor(tsim));
 //        MBSdata->qd[id]      = My_PI*omega*sin(omega*tsim + My_PI);
 //        MBSdata->qdd[id]     = My_PI*omega*omega*cos(omega*tsim + My_PI);
 //    }
 //    else
 //    {
 //        MBSdata->q[id]   = My_PI*(floor(tsim));
 //        MBSdata->qd[id]  = 0;
 //        MBSdata->qdd[id] = 0;
 //    }

    //Rear Left Leg
 //    id = R2_RL;
	// if(fmod(tsim,2*T) < T)
	// {
 //        MBSdata->q[id]    	= My_PI*(-cos(omega*tsim)+1+floor(tsim));
 //        MBSdata->qd[id]      = My_PI*omega*sin(omega*tsim);
 //        MBSdata->qdd[id]     = My_PI*omega*omega*cos(omega*tsim);
 //    }
 //    else
 //    {
 //        MBSdata->q[id]   = My_PI*(1+floor(tsim));
 //        MBSdata->qd[id]  = 0;
 //        MBSdata->qdd[id] = 0;
 //   }
#else
		//Front Right Leg
//    id = R2_FR;
//	    MBSdata->q[id]    	= 0;
//        MBSdata->qd[id]  = 0;
//        MBSdata->qdd[id] = 0;
//
//
//    //Front Left Leg
//    id = R2_FL;
//  	    MBSdata->q[id]    	= 0;
//        MBSdata->qd[id]  = 0;
//        MBSdata->qdd[id] = 0;
//
//    //Rear Right Leg
//    id = R2_RR;
// 	    MBSdata->q[id]    	= 0;
//        MBSdata->qd[id]  = 0;
//        MBSdata->qdd[id] = 0;
//
//    //Rear Left Leg
//    id = R2_RL;
//	    MBSdata->q[id]    	= 0;
//        MBSdata->qd[id]  = 0;
//        MBSdata->qdd[id] = 0;
//

#endif

}
