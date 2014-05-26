//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2006 
// Last update : 01/10/2008 
//--------------------------- 

#include "simu_def.h"

void point_contact_model(double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4],
					   MBSdataStruct *MBSdata, double tsim,
					   int ixF, double *dxF, double *SWr);

double* user_ExtForces(double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4], 
					   double AxF[4], double OMPxF[4], 
					   MBSdataStruct *MBSdata, double tsim, int ixF)
{

	double Fx=0.0, Fy=0.0, Fz=0.0;
	double Mx=0.0, My=0.0, Mz=0.0;
	double dxF[3+1] ={0.0, 0.0, 0.0, 0.0}; // +1 because indexes begin at 1
	int i;

	#ifdef SIMBODY
	SimbodyBodiesStruct *simbodyBodies;
	#endif

	double *SWr = MBSdata->SWr[ixF];
	int idpt = 0;
        
    // user variables
    UserIOStruct *uvs;
    
    uvs = MBSdata->user_IO;

	idpt = MBSdata->xfidpt[ixF];

	dxF[1] = MBSdata->dpt[1][idpt];
	dxF[2] = MBSdata->dpt[2][idpt];
	dxF[3] = MBSdata->dpt[3][idpt];

	#ifdef SIMBODY

	if(ixF == 1)  //compute all the forces at once (arbitrary for 1st external force) (else compute same thing for each force sensors) 
	{
	// 1) Simbody receives kinematics from Robotran
    update_simbody_kinematics(MBSdata->user_IO->simbodyStruct->simbodyBodies, MBSdata);
        
    // 2) Simbody computes contact force
    loop_Simbody(MBSdata->user_IO->simbodyStruct);
	}

	// 3) Simbody sends contact force to robotran dynamics
	simbodyBodies = uvs->simbodyStruct->simbodyBodies;	
	for(i=0; i<simbodyBodies->nb_contact_bodies; i++)
	{
	 	if (ixF == simbodyBodies->F_sensor_Robotran_index[i])
	 	{
	 		Fx += simbodyBodies->force_bodies[i][0]; 
	 		Fy += simbodyBodies->force_bodies[i][1]; 
	 		Fz += simbodyBodies->force_bodies[i][2]; 

	 		Mx += simbodyBodies->torque_bodies[i][0]; 
	 		My += simbodyBodies->torque_bodies[i][1]; 
	 		Mz += simbodyBodies->torque_bodies[i][2];  		

	 		break;
	 	}
	}

	#endif

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

	#ifdef GCM_MT
		// Compute thing resultant forces and moments from the ground
		point_contact_model(PxF, RxF, VxF, OMxF, MBSdata, tsim, ixF, dxF, SWr);
	#endif
	

	return SWr;
}

void point_contact_model(double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4],
					   MBSdataStruct *MBSdata, double tsim,
					   int ixF, double *dxF, double *SWr)
{

	double Fx=0.0, Fy=0.0, Fz=0.0;
	double Mx=0.0, My=0.0, Mz=0.0;


	if(PxF[3] < 0)
	{
		Fz = 10000.0*(-PxF[3]) - 100.0*VxF[3];
		Fx = (-2000.9)*VxF[1];
	}

	SWr[1] = Fx;
	SWr[3] = Fz;
}
