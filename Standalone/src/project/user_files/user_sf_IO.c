/*===========================================================================*
 *
 *  user_sf_IO.c
 * 
 *  Generation date: Thu Jul 10 23:18:37 2014

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
    int i;
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

    // Voltage //
    for (i=1;i<=3;i++)
    {
        uvs->Voltage[i] = 0.0;
    }

    // refs //
    for (i=1;i<=3;i++)
    {
        uvs->refs[i] = 0.0;
    }

    // servo_type //
    for (i=1;i<=3;i++)
    {
        uvs->servo_type[i] = 0;
    }

    // cvs //
    uvs->cvs = init_ControllerStruct();

    // simbodyStruct //
    uvs->simbodyStruct = init_SimbodyStruct();

    // actuatorsStruct //
    uvs->actuatorsStruct = init_ActuatorsStruct();

    return uvs;
}


void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s)
{

    // ControllerStruct: cvs //
    free_ControllerStruct(uvs->cvs);

    // SimbodyStruct: simbodyStruct //
    free_SimbodyStruct(uvs->simbodyStruct);

    // ActuatorsStruct: actuatorsStruct //
    free_ActuatorsStruct(uvs->actuatorsStruct);

    free(uvs);
}

