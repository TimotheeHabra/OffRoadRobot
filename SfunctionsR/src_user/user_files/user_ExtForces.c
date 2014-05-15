//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2006 
// Last update : 01/10/2008 
//--------------------------- 

#include "simu_def.h"

double* user_ExtForces(double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4], 
					   double AxF[4], double OMPxF[4], 
					   MBSdataStruct *MBSdata, double tsim, int ixF)
{
	int i;
	double Fx=0.0, Fy=0.0, Fz=0.0;
	double Mx=0.0, My=0.0, Mz=0.0;
	double dxF[3+1] ={0.0, 0.0, 0.0, 0.0}; // +1 because indexes begin at 1

	double *SWr = MBSdata->SWr[ixF];
	int idpt = 0;
        
    // user variables
    UserIOStruct *uvs;
    
    uvs = MBSdata->user_IO;

	idpt = MBSdata->xfidpt[ixF];

	dxF[1] = MBSdata->dpt[1][idpt];
	dxF[2] = MBSdata->dpt[2][idpt];
	dxF[3] = MBSdata->dpt[3][idpt];

	// initializes the terms of Swr
	SWr[1] = Fx;
	SWr[2] = Fy;
	SWr[3] = Fz;
	SWr[4] = Mx;
	SWr[5] = My;
	SWr[6] = Mz;
	SWr[7] = dxF[1];
	SWr[8] = dxF[2];
	SWr[9] = dxF[3];

	return SWr;
}

