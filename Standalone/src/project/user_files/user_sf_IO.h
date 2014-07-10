/*===========================================================================*
 *
 *  user_sf_IO.h
 *
 *  Generation date: Thu Jul 10 17:10:38 2014

 *
 *  (c) Universite catholique de Louvain
 *      Departement de Mecanique
 *      Unite de Production Mecanique et Machines
 *      2, Place du Levant
 *      1348 Louvain-la-Neuve
 *  http://www.robotran.be//
 *
/*===========================================================================*/

#ifndef UsersfIO_h
#define UsersfIO_h
/*--------------------*/

#include "userDef.h"
#include "ControllersStruct.h"

typedef struct UserIOStruct
{
    double tsim_out1;
    double output1[10+1];
    double output2[10+1];
    double Voltage[3+1];
    double refs[3+1];
    int servo_type[3+1];
    ControllerStruct *cvs;
    SimbodyStruct *simbodyStruct;
    SEActuatorStruct *acs[4];

} UserIOStruct;

/*--------------------*/
#endif
