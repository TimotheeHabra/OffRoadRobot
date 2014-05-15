/*===========================================================================*
  *
  *  user_sf_IO.h
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

#ifndef UsersfIO_h
#define UsersfIO_h
/*--------------------*/
 
#ifdef ACCELRED 
#define S_FUNCTION_NAME  mbs_sf_accelred_OffRoadRobot 
#elif defined DIRDYNARED 
#define S_FUNCTION_NAME  mbs_sf_dirdynared_OffRoadRobot 
#elif defined INVDYNARED 
#define S_FUNCTION_NAME  mbs_sf_invdynared_OffRoadRobot 
#elif defined SENSORKIN 
#define S_FUNCTION_NAME  mbs_sf_sensorkin_OffRoadRobot 
#endif 
 
#define SF_N_USER_INPUT 0 
#define SF_N_USER_OUTPUT 3 

#include "userDef.h"
#include "ControllersStruct.h"
 
typedef struct UserIOStruct 
{
    double tsim_out;
    double output1[10+1];
    double output2[10+1];
    ControllerStruct *cvs;

} UserIOStruct;

/*--------------------*/
#endif
