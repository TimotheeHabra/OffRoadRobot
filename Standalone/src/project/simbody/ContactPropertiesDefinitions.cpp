#if defined(SIMBODY) & defined(__cplusplus)

#include "simbody_cpp_functions.h"
#include "SimbodyStruct.h"
#include <string>

int fill_bodies_contact_properties(ContactPropertiesStruct* BodyContProp, int NumberofBodies)
{
	int i;
	ContactPropertiesStruct* CurBodyContProp;
	for(i=0;i<NumberofBodies;i++) // here we have 4 whels with the same properties so I'll fill this structure in a loop
	{
		CurBodyContProp = &(BodyContProp[i]);
		// set all the mechanical parameters of the contact
		CurBodyContProp->ud = 0.9;   // dynamic   dry friction coefficient
		CurBodyContProp->us = 1.1;   // static;  it is required ud < us
		CurBodyContProp->uv = 0;     // viscous (force/velocity)
		CurBodyContProp->c = 2; // dissipation (1/v)
		CurBodyContProp->k = 1e6; // stiffness (pascals)
		CurBodyContProp->thickness = 0.008;
		
		CurBodyContProp->Geometry = 1;
		CurBodyContProp->ScaleFactor = 1;
		
		CurBodyContProp->Transform[0] = 0;
		CurBodyContProp->Transform[1] = -0.015;
		CurBodyContProp->Transform[2] = -0.032;

		CurBodyContProp->Rotation[0] = 0; 
		CurBodyContProp->Rotation[1] = -90;
		CurBodyContProp->Rotation[2] = 0;

		sprintf(CurBodyContProp->FileName, "Leg_AllTerrainRobot.obj"); // file should be in the folder \Standalone\src\project\simbody
	}
	return 0;
}

int fill_ground_contact_properties(ContactPropertiesStruct* BodyContProp)
{
	// set all the mechanical parameters of the contact
	BodyContProp->ud = 0.9;   // dynamic   dry friction coefficient
	BodyContProp->us = 1.1;   // static;  it is required ud < us
	BodyContProp->uv = 0;     // viscous (force/velocity)
	BodyContProp->c = 1e3;   // dissipation (1/v)
	BodyContProp->k = 5e3;    // stiffness (pascals)
	BodyContProp->thickness = 0.01;
	
	BodyContProp->Geometry = 1; // 0 means OneHalfSpace  z+; 1 means a mesh

	BodyContProp->ScaleFactor = 0.5;
	
	BodyContProp->Transform[0] = -2.5;
	BodyContProp->Transform[1] = -2;
	BodyContProp->Transform[2] = -0.05;

	BodyContProp->Rotation[0] = 90;
	BodyContProp->Rotation[1] = 0;
	BodyContProp->Rotation[2] = 0;
	
	sprintf(BodyContProp->FileName, "ground_mine.obj");  // file should be in the folder \Standalone\src\project\simbody
	

	return 0;
}
#endif
