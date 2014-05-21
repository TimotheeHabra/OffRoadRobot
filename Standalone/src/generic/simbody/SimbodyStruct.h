#ifndef __SIMBODY_STRUCT_H__
#define __SIMBODY_STRUCT_H__

#ifdef SIMBODY

#include "SimbodyBodiesStruct.h"

typedef struct SimbodyStruct
{
	SimbodyBodiesStruct* simbodyBodies;

} SimbodyStruct;


#else

typedef struct SimbodyStruct
{
	int nothing;
} SimbodyStruct;

#endif

SimbodyStruct *init_SimbodyStruct(void);
void free_SimbodyStruct(SimbodyStruct*);

#endif
