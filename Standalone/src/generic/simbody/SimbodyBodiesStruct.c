#ifdef SIMBODY

#include "SimbodyBodiesStruct.h"
//#include "useful_functions.h"

SimbodyBodiesStruct *init_SimbodyBodiesStruct()
{
	int i, j;
	int S_Sensors_array[NB_CONTACT_BODIES] = S_SENSORS_ARRAY;
	int F_Sensors_array[NB_CONTACT_BODIES] = F_SENSORS_ARRAY;

	SimbodyBodiesStruct *simbodyBodiesStruct;

	simbodyBodiesStruct = (SimbodyBodiesStruct*) malloc(sizeof(SimbodyBodiesStruct));

	simbodyBodiesStruct->nb_contact_bodies = NB_CONTACT_BODIES;

	for(i=0; i<NB_CONTACT_BODIES; i++)
	{
		simbodyBodiesStruct->S_sensor_Robotran_index[i] = S_Sensors_array[i];
		simbodyBodiesStruct->F_sensor_Robotran_index[i] = F_Sensors_array[i];
		simbodyBodiesStruct->Simbody_index[i]           = 1;
		
		for(j=0; j<3; j++)
		{
			simbodyBodiesStruct->abs_pos[i][j]   = 0.0;
			simbodyBodiesStruct->lin_vel[i][j]   = 0.0;
			simbodyBodiesStruct->ang_vel[i][j]   = 0.0;
			simbodyBodiesStruct->force_bodies[i][j]  = 0.0;
			simbodyBodiesStruct->torque_bodies[i][j] = 0.0;
		}

		for(j=0; j<9; j++)
		{
			simbodyBodiesStruct->rot_matrix[i][j] = 0.0;
		}

		simbodyBodiesStruct->rot_matrix[i][0] = 1.0; simbodyBodiesStruct->rot_matrix[i][4] = 1.0; simbodyBodiesStruct->rot_matrix[i][8] = 1.0; // Z: to have identical rotation - not a singular one

	}

	fill_bodies_contact_properties(simbodyBodiesStruct->BodyContProp, NB_CONTACT_BODIES);
    fill_ground_contact_properties(&simbodyBodiesStruct->GroundContProp);

	return simbodyBodiesStruct;

}


void free_SimbodyBodiesStruct(SimbodyBodiesStruct *simbodyBodiesStruct)
{
	free(simbodyBodiesStruct);
}

#endif
