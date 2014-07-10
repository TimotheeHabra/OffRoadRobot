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

ActuatorsStruct* init_ActuatorsStruct(void)
{
    
    ActuatorsStruct* actuatorsStruct;
    actuatorsStruct = (ActuatorsStruct*) malloc(sizeof(ActuatorsStruct));


    actuatorsStruct->acs = (SEActuatorStruct *) malloc(NB_ACTUATED_JOINTS_TMP*sizeof(ActuatorsStruct));
    init_SEActuatorStruct(actuatorsStruct->acs); 
    //actuatorsStruct->acs = init_SEActuatorStruct(); 
    return actuatorsStruct;
    return actuatorsStruct;
}

void free_ActuatorsStruct(ActuatorsStruct* actuatorsStruct)
{

    free_SEActuatorStruct(actuatorsStruct->acs);

    free(actuatorsStruct);

}

// SEActuatorStruct
void init_SEActuatorStruct(SEActuatorStruct **acs)
//SEActuatorStruct** init_SEActuatorStruct(void)
{
    int i;
    int type;
    //SEActuatorStruct* acs[NB_ACTUATED_JOINTS_TMP];
    int actuator_type_array[NB_ACTUATED_JOINTS_TMP] = ACTUATOR_TYPE_ARRAY;

    //acs = malloc(4 * sizeof(SEActuatorStruct *));

    //if(acs == NULL)
    //    printf("acs malloc return NULL\n\n");
    //else
    printf("acs malloc return OK\n\n");
        
    for (i=0; i < NB_ACTUATED_JOINTS_TMP; i++)
    {
        type = actuator_type_array[i];
        acs[i] = (SEActuatorStruct*) malloc(sizeof(SEActuatorStruct));

        switch (type) {
          case 1:
            (*acs[i]).Resistance=0.70;
            (*acs[i]).GearRatio=10;
            (*acs[i]).Damping=1;
            (*acs[i]).Inductance=0.000265;
            (*acs[i]).Inertia= 0.01;
            (*acs[i]).Kbemf= 0.00261;
            (*acs[i]).SeriesDamping=0.1;
            (*acs[i]).SeriesSpring=1000.0;
            (*acs[i]).TrqConst=0.00261;
            (*acs[i]).Isaturation=50;
            (*acs[i]).Vsaturation=40;
            //sprintf((*acs[i]).type,"Medium");
            printf("actuator type is 1 \n");
            break;
          case 2:
            (*acs[i]).Resistance=1.0;
            (*acs[i]).GearRatio=1.0;
            (*acs[i]).Damping=1.0;
            (*acs[i]).Inductance=0.002;
            (*acs[i]).Inertia= 0.01;
            (*acs[i]).Kbemf= 0.08;
            (*acs[i]).SeriesDamping=0.2;
            (*acs[i]).SeriesSpring=2000.0;
            (*acs[i]).TrqConst=0.08;
            (*acs[i]).Isaturation=50;
            (*acs[i]).Vsaturation=80;
            //sprintf((*acs[i]).type,"Big");
            printf("actuator type is 2 \n");
            break;
          default:
            (*acs[i]).Resistance=1.0;
            (*acs[i]).GearRatio=100;
            (*acs[i]).Damping=1.0;
            (*acs[i]).Inductance=0.002;
            (*acs[i]).Inertia= 0.01;
            (*acs[i]).Kbemf= 0.08;
            (*acs[i]).SeriesDamping=0.2;
            (*acs[i]).SeriesSpring=2000.0;
            (*acs[i]).TrqConst=0.08;
            (*acs[i]).Isaturation=50;
            (*acs[i]).Vsaturation=80;
            //sprintf((*acs[i]).type,"Big");
            printf("detault actuator selected \n");
          }
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
void free_SEActuatorStruct(SEActuatorStruct *acs[])
{

    int i;

    for (i=0; i < NB_ACTUATED_JOINTS_TMP; i++)
    {
        free(acs[i]);
    }

}

// ActuatorStruct
void free_PActuatorStruct(PActuatorStruct *pacs)
{
    free(pacs);
}
