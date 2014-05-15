//---------------------------
// Nicolas Van der Noot
//
// Creation : 19-Sep-2013
// Last update : 06-May-2014
//---------------------------

#include <stdlib.h>

#include "ControllersStruct.h"


// ---- Controlleres initialization ---- //

// ControllerStruc
ControllerStruct * init_ControllerStruct(void)
{
    ControllerStruct *cvs;

    int i;

    cvs = (ControllerStruct*) malloc(sizeof(ControllerStruct));

    cvs->t = 0.0;

    for (i=0;i<4;i++)
    {
		cvs->Control[i] = 0.0;
    }

    return cvs;
}

// ---- Controllers: free ---- //

// ControllerStruc
void free_ControllerStruct(ControllerStruct *cvs)
{
    free(cvs);
}
