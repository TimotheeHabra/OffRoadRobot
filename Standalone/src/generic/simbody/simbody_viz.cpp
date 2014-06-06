#if defined(SIMBODYVIZ) & defined(__cplusplus)

#include "cmake_config.h"
#include "simbody_cpp_functions.h"
#include "SimbodyStruct.h"
#include "simbody_viz.h"

/////////////////////////////////////
//function used by C and C++ code (must only use C compatible input/output types)
/////////////////////////////////////

void reportSimbody(void* pv_simbodyVariables){

	SimbodyVariables* p_simbodyVariables = (SimbodyVariables*)pv_simbodyVariables;
	#ifdef SIMBODYVIZ
		 p_simbodyVariables->p_viz->report(*(p_simbodyVariables->p_state));
    #endif
}

#endif
