#ifdef SIMBODY

#include "SimbodyBodiesStruct.h"
#include "useful_functions.h"

SimbodyBodiesStruct *init_SimbodyBodiesStruct()
{
	int i, j;


	SimbodyBodiesStruct *simbodyBodiesStruct;

	simbodyBodiesStruct = (SimbodyBodiesStruct*) malloc(sizeof(SimbodyBodiesStruct));

	simbodyBodiesStruct->nb_contact_bodies = NB_CONTACT_BODIES;

	
	simbodyBodiesStruct->S_sensor_Robotran_index[0] = S_FR_Sensor;
	simbodyBodiesStruct->F_sensor_Robotran_index[0] = F_FR_Sensor;
	simbodyBodiesStruct->Simbody_index[0]           = 1;

	simbodyBodiesStruct->S_sensor_Robotran_index[1] = S_FL_Sensor;
	simbodyBodiesStruct->F_sensor_Robotran_index[1] = F_FL_Sensor;
	simbodyBodiesStruct->Simbody_index[1]           = 2;

	simbodyBodiesStruct->S_sensor_Robotran_index[2] = S_RL_Sensor;
	simbodyBodiesStruct->F_sensor_Robotran_index[2] = F_RL_Sensor;
	simbodyBodiesStruct->Simbody_index[2]           = 3;

	simbodyBodiesStruct->S_sensor_Robotran_index[3] = S_RR_Sensor;
	simbodyBodiesStruct->F_sensor_Robotran_index[3] = F_RR_Sensor;
	simbodyBodiesStruct->Simbody_index[3]           = 4;


	for(i=0; i<NB_CONTACT_BODIES; i++)
	{
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

	return simbodyBodiesStruct;

}


void free_SimbodyBodiesStruct(SimbodyBodiesStruct *simbodyBodiesStruct)
{
	free(simbodyBodiesStruct);
}

#endif
