//---------------------------
// Nicolas Van der Noot
//
// Creation : 24-Jan-2014
// Last update : Thu May 08 15:44:53 2014
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

