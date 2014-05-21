#ifndef __SIMBODY_STRUCT_H__
#define __SIMBODY_STRUCT_H__

#ifdef SIMBODY

#include "SimbodyBodiesStruct.h"

typedef struct SimbodyStruct
{
	SimbodyBodiesStruct* simbodyBodies;

	void* p_simbodyVariables;  //type is void* to remain compatible with C code

} SimbodyStruct;


#else

typedef void* SimbodyStruct;

#endif

SimbodyStruct *init_SimbodyStruct(void);
void free_SimbodyStruct(SimbodyStruct*);

#endif
