#ifdef SIMBODY
#ifndef _SIMBODY_CPP_FUNCTIONS_H_
#define _SIMBODY_CPP_FUNCTIONS_H_

/////////////////////////////////////
//function called by C and implemented in C++ (must only use C compatible input/output types)
/////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

	#include "SimbodyBodiesStruct.h"
	#include "SimbodyStruct.h"

	//note : SimbodyVariables* casted in void* for C compatibility
	void* prepare_simbody(SimbodyBodiesStruct* p_simbodyBodiesStruct);  
	int loop_Simbody (SimbodyStruct *simbodyStruct); 
	void free_Simbody(void* p_simbodyVariables);

#ifdef __cplusplus
}
#endif

/////////////////////////////////////
//function used by C++ code only
/////////////////////////////////////
#ifdef __cplusplus

	#include "Simbody.h"	
	#include <cstdio>
	#include <exception>
	#include <algorithm>
	#include <iostream>
	#include <fstream>
	#include <string>
		
	using namespace SimTK;
	
	typedef struct SimbodyVariables SimbodyVariables;
	struct SimbodyVariables
	{
		MultibodySystem *p_system;
		SimbodyMatterSubsystem *p_matter;
		ContactTrackerSubsystem  *p_tracker; 
	    CompliantContactSubsystem *p_contactForces;
		State *p_state;
	#ifdef SIMBODYVIZ
			Visualizer *p_viz;
	#endif
	};
	
	 int init_Simbody(SimbodyVariables *p_simbodyVariables, SimbodyBodiesStruct *p_simbodyBodiesStruct);

#endif
 
#endif
#endif
