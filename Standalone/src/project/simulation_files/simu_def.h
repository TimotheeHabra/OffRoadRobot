//---------------------------
// Creation : ../../2013
// Last update : ../../2013
//---------------------------

#ifndef simulation_def_h
#define simulation_def_h
//--------------------*/

#include <math.h>
#include <stdlib.h>

#include "user_sf_IO.h"
#include "MBSdef.h"
#include "MBSfun.h"
#include "MBSdataStruct.h"
#include "ControllersStruct.h"
#include "nrutil.h"

#ifdef SIMBODY
#include "simbody_cpp_functions.h"
#include "simbody_functions.h"
#endif


// ---- Constants & Macros ---- //

// Actuator Options:
#define Act_order 1 // 1st (electric), 2nd (mechanical) or 3rd (electrical, mechanical)
#define Act_type 1  // 0-> SEA-Small, 1 ->SEA-Med, 2->SEA-Big, 3-> PEA-default


// motor indices
#define  M_FR 0
#define  M_FL 1
#define  M_RR 2
#define  M_RL 3

// driven joints (4 legs)
#define R2_FR 7
#define R2_FL 9
#define R2_RR 11
#define R2_RL 13

//spring joints
#define Spring_FR 8
#define Spring_FL 10
#define Spring_RR 12
#define Spring_RL 14

// ---- Custom Functions ---- //

void controller_init(ControllerStruct *cvs);
void controller_loop(ControllerStruct *cvs);

double limit_angle(double angle);

void controller_inputs(MBSdataStruct *MBSdata);
void controller_outputs(MBSdataStruct *MBSdata);

void simu_outputs(MBSdataStruct *MBSdata);

void simu_controller_loop(MBSdataStruct *MBSdata);

/*--------------------*/
#endif
