//---------------------------
// Nicolas Van der Noot
//
// Creation : 24-Jan-2014
// Last update : Wed May 21 16:40:42 2014
//---------------------------

#include <stdlib.h>

#include "ControllersStruct.h"


// ---- Controllers initialization ---- //
 
// ControllerInputsStruct
ControllerInputsStruct * init_ControllerInputsStruct(void)
{
    ControllerInputsStruct *cvs;

    cvs = (ControllerInputsStruct*) malloc(sizeof(ControllerInputsStruct));

    cvs->tsim = 0.0;

    return cvs;
}

// ControllerOutputsStruct
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

// ControllerStruct
ControllerStruct * init_ControllerStruct(void)
{
    ControllerStruct *cvs;

    cvs = (ControllerStruct*) malloc(sizeof(ControllerStruct));

    cvs->Inputs = init_ControllerInputsStruct();

    cvs->Outputs = init_ControllerOutputsStruct();

    return cvs;
}

// ---- Controllers: free ---- //

// ControllerInputsStruct
void free_ControllerInputsStruct(ControllerInputsStruct *cvs)
{
    free(cvs);
}

// ControllerOutputsStruct
void free_ControllerOutputsStruct(ControllerOutputsStruct *cvs)
{
    free(cvs);
}

// ControllerStruct
void free_ControllerStruct(ControllerStruct *cvs)
{
    free_ControllerInputsStruct(cvs->Inputs);

    free_ControllerOutputsStruct(cvs->Outputs);

    free(cvs);
}

