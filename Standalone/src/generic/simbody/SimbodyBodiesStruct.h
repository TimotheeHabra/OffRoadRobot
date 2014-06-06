#ifndef __SIMBODY_BODIES_STRUCT_H__
#define __SIMBODY_BODIES_STRUCT_H__

#ifdef SIMBODY

#include "SimbodyBodiesDefinitions.h"

#define LENGTHOFFILENAME 150

typedef struct ContactPropertiesStruct
{
	double ud; // dynamic   dry friction coefficient- it is the same as in your GCM
    double us; // static;  it is required ud < us
    double uv;  // viscous (force/velocity)
    double c;//3.3; // dissipation (1/v)
	double k;
	double thickness;
	double ScaleFactor; //how to scale a mesh
	double Rotation[3]; //how to rotate a mesh
	double Transform[3]; //how to shift a mesh
	char FileName[LENGTHOFFILENAME];
	int Geometry; // 0 means OneHalfSpace; 1 means mesh
} ContactPropertiesStruct;

typedef struct SimbodyBodiesStruct
{
	int nb_contact_bodies;

	int F_sensor_Robotran_index[NB_CONTACT_BODIES];
	int S_sensor_Robotran_index[NB_CONTACT_BODIES];
	
	ContactPropertiesStruct BodyContProp[NB_CONTACT_BODIES];
	ContactPropertiesStruct GroundContProp;
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

int fill_bodies_contact_properties(ContactPropertiesStruct* BodyContProp, int NumberofBodies);
int fill_ground_contact_properties(ContactPropertiesStruct* GroundContProp);

#endif

#endif
