
#include "SimbodyStruct.h"
#include "useful_functions.h"

#ifdef SIMBODY

SimbodyStruct *init_SimbodyStruct()
{

	SimbodyStruct *simbodyStruct;

	simbodyStruct = (SimbodyStruct*) malloc(sizeof(SimbodyStruct));

	simbodyStruct->simbodyBodies = init_SimbodyBodiesStruct();

	return simbodyStruct;

}


void free_SimbodyStruct(SimbodyStruct* simbodyStruct)
{
	free_SimbodyBodiesStruct(simbodyStruct->simbodyBodies);
	free(simbodyStruct);
}

#else

SimbodyBodiesStruct *init_SimbodyStruct()
{
	return NULL;
}

void free_SimbodyStruct(SimbodyBodiesStruct* simbodyStruct)
{
}

#endif
