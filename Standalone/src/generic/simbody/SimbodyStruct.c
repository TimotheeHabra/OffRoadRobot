
#include "SimbodyStruct.h"
#include "simbody_cpp_functions.h"
#include "useful_functions.h"

#ifdef SIMBODY

SimbodyStruct *init_SimbodyStruct()
{

	SimbodyStruct *simbodyStruct;
	simbodyStruct = (SimbodyStruct*) malloc(sizeof(SimbodyStruct));

	simbodyStruct->simbodyBodies = init_SimbodyBodiesStruct();

	simbodyStruct->p_simbodyVariables = prepare_simbody(simbodyStruct->simbodyBodies);

	return simbodyStruct;

}


void free_SimbodyStruct(SimbodyStruct* simbodyStruct)
{
	free_SimbodyBodiesStruct(simbodyStruct->simbodyBodies);
	free_Simbody(simbodyStruct->p_simbodyVariables);

	free(simbodyStruct);
}

#else

SimbodyStruct *init_SimbodyStruct()
{
	return NULL;
}

void free_SimbodyStruct(SimbodyStruct* simbodyStruct)
{
}

#endif
