//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2006 
// Last update : 01/10/2008 
//--------------------------- 
 
#include "math.h" 
 
#include "MBSdef.h" 
#include "MBSdataStruct.h" 
 
 
double* user_Link3DForces(double PxF[4], double RxF[4][4],  
					      double VxF[4], double OMxF[4],  
					      double AxF[4], double OMPxF[4],  
					      MBSdataStruct *MBSdata, double tsim,int ixF) 
{ 
	double Fx=0.0, Fy=0.0, Fz=0.0; 
	double Mx=0.0, My=0.0, Mz=0.0; 
 
	double *SWr = MBSdata->l3DWr[ixF]; 
 
/* Begin of user declaration */ 
	 
	 
 
/* End of user declaration */ 
 
 
	switch(ixF){ 
 
/* Begin of user code */ 
		case 1:  
 
					 
		break; 
 
	/*	case 2:  
	 
      
        break; 
     */ 
		 
/* End of user code */ 
 
	} 
	SWr[1]=Fx; 
	SWr[2]=Fy; 
	SWr[3]=Fz; 
	SWr[4]=Mx; 
	SWr[5]=Mz; 
	SWr[6]=Mz; 
 
	return SWr; 
} 
