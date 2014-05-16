//---------------------------
// Creation : ../../2013
// Last update : ../../2013
//---------------------------

#include "controller_def.h"

#define GO

void simple_control(ControllerStruct *cvs)
{

#ifdef GO

	double T 		= 1.0; //[s]
	double My_PI 	= 3.1415926;
	double omega 	= 2*My_PI/(2*T); //[rad/s]

	double tsim 	= cvs->Inputs->tsim;

	//Front Right Leg
	if(fmod(tsim,2*T) < T)
	{
        cvs->Outputs->q_ref[0]    	= My_PI*(-cos(omega*tsim)+1+floor(tsim));
        cvs->Outputs->qd_ref[0]     = My_PI*omega*sin(omega*tsim);
        cvs->Outputs->qdd_ref[0]	= My_PI*omega*omega*cos(omega*tsim);
    }
    else
    {
        cvs->Outputs->q_ref[0]   	= My_PI*(1+floor(tsim));
        cvs->Outputs->qd_ref[0]  	= 0;
        cvs->Outputs->qdd_ref[0] 	= 0;
    }

    //Front Left Leg
    if(fmod(tsim,2*T) >= T)
	{
        cvs->Outputs->q_ref[1]    	= My_PI*(-cos(omega*tsim + My_PI)+floor(tsim));
        cvs->Outputs->qd_ref[1]     = My_PI*omega*sin(omega*tsim + My_PI);
        cvs->Outputs->qdd_ref[1]    = My_PI*omega*omega*cos(omega*tsim + My_PI);
    }
    else
    {
        cvs->Outputs->q_ref[1]  	= My_PI*(floor(tsim));
        cvs->Outputs->qd_ref[1]  	= 0;
        cvs->Outputs->qdd_ref[1] 	= 0;
    }

    //Rear Right Leg
    if(fmod(tsim,2*T) >= T)
	{
        cvs->Outputs->q_ref[2]    	= My_PI*(-cos(omega*tsim + My_PI)+floor(tsim));
        cvs->Outputs->qd_ref[2]     = My_PI*omega*sin(omega*tsim + My_PI);
        cvs->Outputs->qdd_ref[2]    = My_PI*omega*omega*cos(omega*tsim + My_PI);
    }
    else
    {
        cvs->Outputs->q_ref[2]  	= My_PI*(floor(tsim));
        cvs->Outputs->qd_ref[2]  	= 0;
        cvs->Outputs->qdd_ref[2] 	= 0;
    }

    //Rear Left Leg
	if(fmod(tsim,2*T) < T)
	{
        cvs->Outputs->q_ref[3]    	= My_PI*(-cos(omega*tsim)+1+floor(tsim));
        cvs->Outputs->qd_ref[3]     = My_PI*omega*sin(omega*tsim);
        cvs->Outputs->qdd_ref[3]	= My_PI*omega*omega*cos(omega*tsim);
    }
    else
    {
        cvs->Outputs->q_ref[3]   	= My_PI*(1+floor(tsim));
        cvs->Outputs->qd_ref[3]  	= 0;
        cvs->Outputs->qdd_ref[3] 	= 0;
    }

#else  //No control at all. bloc the joints

    int i = 0;

	for (i=0;i<4;i++)
    {
		cvs->Outputs->q_ref[i] = 0.0;
		cvs->Outputs->qd_ref[i] = 0.0;
		cvs->Outputs->qdd_ref[i] = 0.0;
    }

#endif
     
}
