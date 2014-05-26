#if defined(SIMBODY) & defined(__cplusplus)

#include "cmake_config.h"
#include "simbody_cpp_functions.h"
#include "SimbodyStruct.h"

////////////////////////////////////
//function used by C++ code only
/////////////////////////////////////

int init_Simbody(SimbodyVariables *p_simbodyVariables, SimbodyBodiesStruct *p_simbodyBodiesStruct)
{
try
  { 
	// set all the mechanical parameters of the contact
	const Real ud = 0.9; // dynamic   dry friction coefficient- it is the same as in your GCM
    const Real us = 1.1; // static;  it is required ud < us
    const Real uv = 0;  // viscous (force/velocity)
    const Real c = 0.1;//3.3; // dissipation (1/v)
	const Real BoxMass = 1.0;	 // kg -> not used
	const Real k = 1e5; // pascals 

	MultibodySystem *p_system = p_simbodyVariables->p_system;
	SimbodyMatterSubsystem *p_matter = p_simbodyVariables->p_matter;
	State *p_state = p_simbodyVariables->p_state;

// this part of the code means that there is a ground z = 0;	
//const Rotation R_zdown(Pi/2.,YAxis);
//p_matter->Ground().updBody().addContactSurface(
//       Transform(R_zdown, Vec3(0,0,0)),
//       ContactSurface(ContactGeometry::HalfSpace(),
 //                      ContactMaterial(k,c,us,ud,uv))); // here we add ground "z+" - is available halfspace.

	// creates a mesh for a ground:
	std::ifstream meshFileGr;
	PolygonalMesh GroundMesh;
		printf("1. Open mesh-file %s ...","ground_mine.obj");
		meshFileGr.open(PROJECT_ABS_PATH"/src/project/simbody/ground_mine.obj"); 
		printf(" succeed! \n");
		printf("2. Load a mesh from Obj-file ... ");
		GroundMesh.loadObjFile(meshFileGr); 
		printf(" succeed! \n");
	    meshFileGr.close();
				
		printf("3. Transform a mesh ... ");
		const Rotation R_0(Pi/2.,XAxis);
		GroundMesh.transformMesh(Transform(R_0,Vec3(-2.5,-8,-0.1)));
		printf(" succeed! \n");

// apply a contactGeometry for Ground
	printf("4. Convert a polygonal mesh to triangle mesh ... ");

	ContactGeometry::TriangleMesh GroundTrM(GroundMesh); 

	printf(" succeed! ");
	printf("Number of faces = %i\n",GroundTrM.getNumFaces());
	printf("5. Add contact surface ... ");
	p_matter->Ground().updBody().addContactSurface(Transform(Vec3(0,0,0)),
							   ContactSurface(GroundTrM,
											   ContactMaterial(k/20,c/100,us,ud,uv),
											   0.001) 
											   );
	printf(" succeed! \n");//*/
#ifdef VIZ
	DecorativeMesh showGround = DecorativeMesh(GroundMesh);
	p_matter->updGround().addBodyDecoration(Transform(Vec3(0,0,0)),showGround);
#endif
   
	const Vec3 comLoc(0, 0, 0);  // Location of the center of mass

// set the mass-inertial properties of the body. These parameters might be arbitrary
	const Inertia centralInertia(Vec3(17,2,16), Vec3(0,0,.2)); // 
 	Body::Rigid WheelBody(MassProperties(BoxMass, comLoc, centralInertia));
	
	// creates a mesh for a wheel:
	std::ifstream meshFile1;
	PolygonalMesh WheelMesh;
		printf("1. Open mesh-file %s ...","Leg_AllTerrainRobot.obj");
		meshFile1.open(PROJECT_ABS_PATH"/src/project/simbody/Leg_AllTerrainRobot.obj"); 
		printf(" succeed! \n");
		printf("2. Load a mesh from Obj-file ... ");
		WheelMesh.loadObjFile(meshFile1); 
		printf(" succeed! \n");
	    meshFile1.close();
				
		printf("3. Transform a mesh ... ");
		//WheelMesh.scaleMesh(0.0254); // (if you need) you can scale the mesh
		const Rotation R_1(-Pi/2.,YAxis);  // or rotate it
		WheelMesh.transformMesh(Transform(R_1,Vec3(0,-0.015,-0.032))); // and move
		printf(" succeed! \n");

// apply a contactGeometry for WheelBody
	printf("4. Convert a polygonal mesh to triangle mesh ... ");

	ContactGeometry::TriangleMesh WheelTrM(WheelMesh); 

	printf(" succeed! ");
	printf("Number of faces = %i\n",WheelTrM.getNumFaces());
	printf("5. Add contact surface ... ");
	WheelBody.addContactSurface(Transform(Vec3(0,0,0)),
							   ContactSurface(WheelTrM,
											   ContactMaterial(k*10,c*20,us,ud,uv),
											   0.008) // thickness, it is a parameter of the Compliant Contact - thickness of the layer of springs
											   );//*/
	printf(" succeed! \n");

// it is only for visualization. Doesn't work under Linux
 	#ifdef VIZ
    //DecorativeMesh showGround = DecorativeMesh(GroundMesh);
	//Ground.addBodyDecoration(showGround);
    DecorativeMesh showBox = DecorativeMesh(WheelMesh);
	WheelBody.addDecoration(Transform(), showBox.setColor(Red).setOpacity(1).setRepresentation(SimTK::DecorativeGeometry::Representation(0)));
    #endif // VIZ


// creates 4 wheels - 4 instances of the WheelBody, each has 6 DOF

	for (int i=0;i<=3;i++)
	{
		MobilizedBody::Free Wheel(p_matter->Ground(), Transform(Vec3(0)),
        WheelBody, Transform(Vec3(0)));
    	p_simbodyBodiesStruct->Simbody_index[i] = Wheel.getMobilizedBodyIndex(); 
	}
	
	}
 catch(const std::exception& e)
	{
		std::cout << e.what();
		std::cout << "Press any key to exit..." << std::endl;
		char extra;

		std::cin >> extra;
		
		exit(0x30B08A); // code of error resembles my surname Zobova :)
	}
 return 0;
}

#endif
