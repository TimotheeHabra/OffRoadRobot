/*===========================================================================*
  *
  *  user_sf_IO.c
  *	
  *  Project:	OffRoadRobot
  * 
  *  Generation date: 06-May-2014 13:30:32
  * 
  *  (c) Universite catholique de Louvain
  *      D�partement de M�canique 
  *      Unit� de Production M�canique et Machines 
  *      2, Place du Levant 
  *      1348 Louvain-la-Neuve 
  *  http://www.robotran.be// 
  *  
 /*===========================================================================*/

#include "MBSfun.h" 
#include "user_sf_IO.h" 
#include "sfdef.h" 
#include "userDef.h"
#include "ControllersStruct.h"


UserIOStruct * initUserIO(MBSdataStruct *s)
{
	UserIOStruct *uvs;
	int i=0;
	//
	uvs = (UserIOStruct*) malloc(sizeof(UserIOStruct));
	

	// tsim_out //
	uvs->tsim_out = 0.0;

	// output1 //
	for (i=1;i<=10;i++)
	{
		uvs->output1[i] = 0.0;
	}

	// output2 //
	for (i=1;i<=10;i++)
	{
		uvs->output2[i] = 0.0;
	}

	// cvs //
	uvs->cvs = init_ControllerStruct();

	return uvs;
}


void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s)
{

	// Controller: cvs //
	free_ControllerStruct(uvs->cvs);

	free(uvs);
}

#ifndef CMEX 
 
void sf_set_user_input_sizes(SimStruct *S, MBSdataStruct *MBSdata, int sf_ninput) 
{ 
	if (SF_N_USER_INPUT > 0) { // warning: index starts at sf_ninput 
        // example: ssSetInputPortWidth(S,sf_ninput,10); 
	} 
} 

void sf_set_user_output_sizes(SimStruct *S, MBSdataStruct *MBSdata) 
        // example: ssSetOutputPortWidth(S, SF_NOUTPUT, 10); 
{ 
	if (SF_N_USER_OUTPUT > 0) { // warning: index starts at SF_NOUTPUT 

		/* User output port0 : tsim_out */ 
		ssSetOutputPortWidth(S, SF_NOUTPUT, 1); 

		/* User output port1 : output1 */ 
		ssSetOutputPortWidth(S, SF_NOUTPUT+1, 10); 

		/* User output port2 : output2 */ 
		ssSetOutputPortWidth(S, SF_NOUTPUT+2, 10); 
	} 
} 

void sf_get_user_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds, int sf_ninput) 
{ 
    // warning: index starts at sf_ninput
    // example: InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,sf_ninput);
    //          MBSdata->user_IO->var1 = *uPtrs0[0];
} 

void sf_set_user_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{ 
    // warning: index starts at SF_NOUTPUT  
    // example: real_T *y0 = ssGetOutputPortRealSignal(S,SF_NOUTPUT); 
    //          *y0 = MBSdata->user_IO->var1;  
    int i;
	real_T *y0 = ssGetOutputPortRealSignal(S,SF_NOUTPUT); 
	real_T *y1 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+1); 
	real_T *y2 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+2); 

	/* User output port0 : tsim_out */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT)) 
      *y0 = MBSdata->user_IO->tsim_out; 

	/* User output port1 : output1 */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+1)) 
      for (i=1;i<=10;i++)
          y1[i-1] = MBSdata->user_IO->output1[i]; 

	/* User output port2 : output2 */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+2)) 
      for (i=1;i<=10;i++)
          y2[i-1] = MBSdata->user_IO->output2[i]; 
} 

#endif 
