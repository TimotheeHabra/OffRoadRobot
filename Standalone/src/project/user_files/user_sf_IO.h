/*===========================================================================*
 *
 *  user_sf_IO.h
 *
 *  Generation date: Wed May 21 16:40:42 2014

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
#include "ActuatorStruct.h"

typedef struct UserIOStruct
{
    double tsim_out1;
    double output1[10+1];
    double output2[10+1];
    double Voltage[NB_ACTUATED_JOINTS]; //motor input voltage (controller)
    double refs[NB_ACTUATED_JOINTS]; //motor input voltage (controller)
    ControllerStruct *cvs;
    SEActuatorStruct *acs;
    int servo_type[NB_ACTUATED_JOINTS];
    SimbodyStruct *simbodyStruct;

} UserIOStruct;

/*--------------------*/
#endif
