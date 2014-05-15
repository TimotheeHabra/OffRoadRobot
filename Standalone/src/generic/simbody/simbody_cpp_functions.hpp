#ifdef SIMBODY
//#define VIZ
#ifndef _SIMBODY_CPP_FUNCTIONS_H_
#define _SIMBODY_CPP_FUNCTIONS_H_


#include "Simbody.h"

#include <cstdio>
#include <exception>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include "SimTKcommon/internal/Mat.h"
#include "SimTKcommon/internal/Rotation.h"
#include "SimTKcommon/internal/Quaternion.h"
#include "simbody/internal/ElasticFoundationForce.h"
#include "simbody/internal/CompliantContactSubsystem.h"
#include "simmath/internal/Contact.h"

extern "C" 
{
  #include "SimbodyBodiesStruct.h"
}


using namespace SimTK;

typedef struct SimbodyVariables SimbodyVariables;
struct SimbodyVariables
{
	MultibodySystem *p_system;
	SimbodyMatterSubsystem *p_matter;
	ContactTrackerSubsystem  *p_tracker; 
    CompliantContactSubsystem *p_contactForces;
	State *p_state;
#ifdef VIZ
		Visualizer *p_viz;
#endif
};

 SimbodyVariables* prepare_simbody(SimbodyBodiesStruct *p_simbodyBodiesStruct);
 int init_Simbody(SimbodyVariables *p_simbodyVariables, SimbodyBodiesStruct *p_simbodyBodiesStruct);

 int loop_Simbody (SimbodyVariables *p_simbodyVariables, SimbodyBodiesStruct* simbodyBodiesStruct);

 void free_Simbody(SimbodyVariables *p_simbodyVariables);
 
#endif
#endif
