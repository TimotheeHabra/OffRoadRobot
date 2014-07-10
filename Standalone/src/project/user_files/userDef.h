//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#ifndef UserDef_h
#define UserDef_h
/*--------------------*/

#include "SimbodyBodiesStruct.h"
#include "SimbodyStruct.h"

// Actuator Options:
// MBSdata->ux size is initilized to the biggest order (3rd) and if the actuator order is lower
// part of the MBSdata->ux array is not used and left zero.
#define Act_order 3 // 1st (electric), 2nd (mechanical) or 3rd (electrical, mechanical)
#define Act_type 1  // 0-> SEA-Small, 1 ->SEA-Med, 2->SEA-Big, 3-> PEA-default
#define NB_ACTUATED_JOINTS 4

/*--------------------*/
#endif
