#ifdef SIMBODYVIZ
//#ifndef _SIMBODY_CPP_FUNCTIONS_H_
//#define _SIMBODY_CPP_FUNCTIONS_H_

/////////////////////////////////////
//function called by C and implemented in C++ (must only use C compatible input/output types)
/////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

	#include "SimbodyBodiesStruct.h"
	#include "SimbodyStruct.h"

	//note : SimbodyVariables* casted in void* for C compatibility
	void reportSimbody(void* p_simbodyVariables);
 //#endif
#ifdef __cplusplus
}
#endif
#endif