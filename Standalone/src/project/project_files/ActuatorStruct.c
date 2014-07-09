//---------------------------
//
// Houman Dallali
//
// Creation : 09.07.2014
// Last update : 09.07.2014
//---------------------------

#include <stdlib.h>
#include <stdio.h>

#include "ActuatorStruct.h"

// ---- Actuator initialization ---- //

// SEActuatorStruct
SEActuatorStruct * init_SEAStruct(int type)
{
    SEActuatorStruct *acs;

    acs = (SEActuatorStruct*) malloc(sizeof(SEActuatorStruct));

    switch (type) {
      case 1:
        acs->Resistance=0.7;
        acs->GearRatio=50;
        acs->Damping=1;
        acs->Inductance=0.000265;
        acs->Inertia= 0.01;
        acs->Kbemf= 0.00261;
        acs->SeriesDamping=0.1;
        acs->SeriesSpring=1000.0;
        acs->TrqConst=0.00261;
        acs->Isaturation=50;
        acs->Vsaturation=40;
        sprintf(acs->type,"Medium");
        printf("actuator type is 1 \n");
        break;
      case 2:
        acs->Resistance=1;
        acs->GearRatio=100;
        acs->Damping=1;
        acs->Inductance=0.002;
        acs->Inertia= 0.01;
        acs->Kbemf= 0.08;
        acs->SeriesDamping=0.2;
        acs->SeriesSpring=2000.0;
        acs->TrqConst=0.08;
        acs->Isaturation=50;
        acs->Vsaturation=80;
        sprintf(acs->type,"Big");
        printf("actuator type is 2 \n");
        break;
      default:
        acs->Resistance=1;
        acs->GearRatio=100;
        acs->Damping=1;
        acs->Inductance=0.002;
        acs->Inertia= 0.01;
        acs->Kbemf= 0.08;
        acs->SeriesDamping=0.2;
        acs->SeriesSpring=2000.0;
        acs->TrqConst=0.08;
        acs->Isaturation=50;
        acs->Vsaturation=80;
        sprintf(acs->type,"Big");
        printf("detault actuator selected \n");
      }

    return acs;
}


// ParallelActuatorStruct
PActuatorStruct * init_PStruct(void)
{
    PActuatorStruct *pacs;

    pacs = (PActuatorStruct*) malloc(sizeof(PActuatorStruct));
        pacs->Resistance=0.5;
        pacs->BallScrewRatio = 29;
        pacs->Damping=1;
        pacs->Inductance=0.002;
        pacs->Inertia= 0.01;
        pacs->Kbemf= 0.04;
        pacs->PDamping=0.1;
        pacs->PSpring=2000.0;
        pacs->TrqConst=0.04;
        pacs->saturation=24;
        printf("default parallel actuator selected \n");

    return pacs;
}

// ActuatorStruct
void free_SEActuatorStruct(SEActuatorStruct *acs)
{
    free(acs);
}

// ActuatorStruct
void free_PActuatorStruct(PActuatorStruct *pacs)
{
    free(pacs);
}
