#ifndef __SIMBODY_BODIES_STRUCT_H__
#define __SIMBODY_BODIES_STRUCT_H__

#ifdef SIMBODY

#include "SimbodyBodiesDefinitions.h"

typedef struct SimbodyBodiesStruct
{
	int nb_contact_bodies;

	int F_sensor_Robotran_index[NB_CONTACT_BODIES];
	int S_sensor_Robotran_index[NB_CONTACT_BODIES];
	int Simbody_index[NB_CONTACT_BODIES];

	double abs_pos[NB_CONTACT_BODIES][3];
	double rot_matrix[NB_CONTACT_BODIES][9];
	double lin_vel[NB_CONTACT_BODIES][3];
	double ang_vel[NB_CONTACT_BODIES][3];

	double force_bodies[NB_CONTACT_BODIES][3];
	double torque_bodies[NB_CONTACT_BODIES][3];

} SimbodyBodiesStruct;

SimbodyBodiesStruct *init_SimbodyBodiesStruct();
void free_SimbodyBodiesStruct(SimbodyBodiesStruct *simbodyBodiesStruct);

#endif

#endif
