/*===========================================================================*
 *
 *  user_sf_IO.c
 * 
 *  Generation date: Thu May 08 15:44:53 2014

 * 
 *  (c) Universite catholique de Louvain
 *      Departement de Mecanique 
 *      Unite de Production Mecanique et Machines 
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
	

    // tsim_out1 //
	uvs->tsim_out1 = 0.0;

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

	// simbodyBodies //
	uvs->simbodyBodies = init_SimbodyBodiesStruct();

	return uvs;
}


void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s)
{

	// Controller: cvs //
	free_ControllerStruct(uvs->cvs);

	// SimbodyBodies: simbodyBodies //
	free_SimbodyBodiesStruct(uvs->simbodyBodies);

	free(uvs);
}

