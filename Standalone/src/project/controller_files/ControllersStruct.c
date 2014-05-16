//---------------------------
// Nicolas Van der Noot
//
// Creation : 24-Jan-2014
// Last update : Fri May 16 15:18:54 2014
//---------------------------

#include <stdlib.h>

#include "ControllersStruct.h"


// ---- Controlleres initialization ---- //
 
// ControllerInputsStruc
ControllerInputsStruct * init_ControllerInputsStruct(void)
{
    ControllerInputsStruct *cvs;

    cvs = (ControllerInputsStruct*) malloc(sizeof(ControllerInputsStruct));

    cvs->tsim = 0.0;

    return cvs;
}

// ControllerOutputsStruc
ControllerOutputsStruct * init_ControllerOutputsStruct(void)
{
    ControllerOutputsStruct *cvs;

    int i;

    cvs = (ControllerOutputsStruct*) malloc(sizeof(ControllerOutputsStruct));

    for (i=0;i<4;i++)
    {
		cvs->q_ref[i] = 0.0;
    }

    for (i=0;i<4;i++)
    {
		cvs->qd_ref[i] = 0.0;
    }

    for (i=0;i<4;i++)
    {
		cvs->qdd_ref[i] = 0.0;
    }

    return cvs;
}

// ControllerStruc
ControllerStruct * init_ControllerStruct(void)
{
    ControllerStruct *cvs;

    cvs = (ControllerStruct*) malloc(sizeof(ControllerStruct));

    cvs->Inputs = init_ControllerInputsStruct();

    cvs->Outputs = init_ControllerOutputsStruct();

    return cvs;
}

// ---- Controllers: free ---- //

// ControllerInputsStruc
void free_ControllerInputsStruct(ControllerInputsStruct *cvs)
{
    free(cvs);
}

// ControllerOutputsStruc
void free_ControllerOutputsStruct(ControllerOutputsStruct *cvs)
{
    free(cvs);
}

// ControllerStruc
void free_ControllerStruct(ControllerStruct *cvs)
{
    free_ControllerInputsStruct(cvs->Inputs);

    free_ControllerOutputsStruct(cvs->Outputs);

    free(cvs);
}

