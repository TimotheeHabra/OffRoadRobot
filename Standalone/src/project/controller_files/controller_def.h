//---------------------------
// Creation : ../../2013
// Last update : ../../2013
//---------------------------

#ifndef controller_def_h
#define controller_def_h
//--------------------*/

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/*
 * Uncomment the ' #include "simstruc.h" ' line to have access
 * to the ' printf ' function in the controller files
 * (the results of the printf will be available in the 'Command Window' of Matlab)
 *
 * This line must be commented before transfering the controller to the real robot
 */
// #include "simstruc.h" 

#include "ControllersStruct.h"

// ---- Constants & Macros ---- //

// ---- Custom Functions ---- //

void controller_init(ControllerStruct *cvs);
void controller_loop(ControllerStruct *cvs);

void simple_control(ControllerStruct *cvs);

/*--------------------*/
#endif
