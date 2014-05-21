#ifndef __SIMBODY_BODIES_STRUCT_H__
#define __SIMBODY_BODIES_STRUCT_H__

#ifdef SIMBODY

// S sensors
#define S_FR_Sensor 1
#define S_FL_Sensor 2
#define S_RL_Sensor 4
#define S_RR_Sensor 3

// F sensors
#define F_FR_Sensor 1
#define F_FL_Sensor 2
#define F_RL_Sensor 4
#define F_RR_Sensor 3

#define NB_CONTACT_BODIES 4

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
