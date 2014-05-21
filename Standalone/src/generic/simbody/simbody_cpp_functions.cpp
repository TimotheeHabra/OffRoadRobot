#if defined(SIMBODY) & defined(__cplusplus)

#include "cmake_config.h"
#include "simbody_cpp_functions.h"
#include "SimbodyStruct.h"

/////////////////////////////////////
//function used by C and C++ code (must only use C compatible input/output types)
/////////////////////////////////////

void* prepare_simbody(SimbodyBodiesStruct *simbodyBodies){

	SimbodyVariables* p_simbodyVariables;
	p_simbodyVariables = new SimbodyVariables;
	
	// set all the mechanical parameters of the contact
    p_simbodyVariables->p_system        = new MultibodySystem;
    p_simbodyVariables->p_matter        = new SimbodyMatterSubsystem(*(p_simbodyVariables->p_system));
    p_simbodyVariables->p_tracker       = new ContactTrackerSubsystem(*(p_simbodyVariables->p_system));
    p_simbodyVariables->p_contactForces = new CompliantContactSubsystem(*(p_simbodyVariables->p_system),*(p_simbodyVariables->p_tracker));

	p_simbodyVariables->p_system->setUpDirection(+ZAxis); // that is for visualization only. The default direction is +X

    //  cout<<"BoxInd in Main = "<<BoxInd<<" -- should be default \n";
    init_Simbody(p_simbodyVariables, simbodyBodies);
    
    //it is "system" commands. We cannot avoid them.    
    #ifdef VIZ	
	 Visualizer viz(system);
	#endif
	p_simbodyVariables->p_system->realizeTopology();

    p_simbodyVariables->p_state = new State (p_simbodyVariables->p_system->getDefaultState());
	
    //it is "system" command. We cannot avoid them. 
    p_simbodyVariables->p_system->realizeModel(*p_simbodyVariables->p_state);

	#ifdef VIZ
		p_simbodyVariables->p_viz = &viz;
	#endif

	return (void*) p_simbodyVariables;
}

int loop_Simbody (SimbodyStruct *simbodyStruct)
{	   
	ContactForce CF;
	int index;
	int i;    
	int ContCount;

	SimbodyVariables* p_simbodyVariables = (SimbodyVariables*)simbodyStruct->p_simbodyVariables;
	SimbodyBodiesStruct* simbodyBodiesStruct = simbodyStruct->simbodyBodies;

	// I'm getting all the system variables 
	MultibodySystem *p_system = p_simbodyVariables->p_system;
	SimbodyMatterSubsystem *p_matter = p_simbodyVariables->p_matter;
	ContactTrackerSubsystem  *p_tracker = p_simbodyVariables->p_tracker; 
    CompliantContactSubsystem *p_contactForces = p_simbodyVariables->p_contactForces;
    State* p_state = p_simbodyVariables->p_state;
	#ifdef VIZ		
	   Visualizer *p_viz = p_simbodyVariables->p_viz;
	#endif
	
	// stage 1: to  update all the cinematic variables for all the bodies in the system

	for(i=0;i<NB_CONTACT_BODIES;i++)
	{
	  MobilizedBodyIndex BodyInd = (MobilizedBodyIndex) simbodyBodiesStruct->Simbody_index[i]; // for each body in the Simbody_index aray
	  const MobilizedBody Box=p_matter->getMobilizedBody(BodyInd); // get the object MobilizedBody 
	  index = i;
	  //to set the coordinates to the bodies, they are stored in Cinematic parameters structure
	   Box.setQToFitTransform(*p_state, Transform(Vec3(simbodyBodiesStruct->abs_pos[index])));
	   Box.setUToFitLinearVelocity (*p_state, Vec3(simbodyBodiesStruct->lin_vel[index]));
	   Box.setUToFitAngularVelocity (*p_state, Vec3(simbodyBodiesStruct->ang_vel[index]));

	    const Mat<3,3,Real> M(simbodyBodiesStruct->rot_matrix[index][0] , simbodyBodiesStruct->rot_matrix[index][1] , simbodyBodiesStruct->rot_matrix[index][2],
	                      simbodyBodiesStruct->rot_matrix[index][3] , simbodyBodiesStruct->rot_matrix[index][4] , simbodyBodiesStruct->rot_matrix[index][5],
		                  simbodyBodiesStruct->rot_matrix[index][6] , simbodyBodiesStruct->rot_matrix[index][7] , simbodyBodiesStruct->rot_matrix[index][8]);
	    const Rotation RotMatr(M, true);
	    const Rotation NewRotMatr=RotMatr.invert();
	    Box.setQToFitRotation(*p_state, NewRotMatr);
		p_system->realize(*p_state,Stage::Dynamics);
		Rotation RBR = Box.getBodyRotation(*p_state);
	
		// to be sure, that without contact all the forces equals zero:		
		for(int j=0; j<3; j++)
		{
				simbodyBodiesStruct->force_bodies[i][j]  = 0.0;
				simbodyBodiesStruct->torque_bodies[i][j] = 0.0;
		} 
	}

//"system" operations - on this stage it computes all the active contacts 	
	p_system->realize(*p_state,Stage::Dynamics);
	p_state->autoUpdateDiscreteVariables();	
	#ifdef VIZ
		p_viz->report(*p_state);
    #endif
	//I'm getting the information about Active Contacts
	int NumofCont = p_contactForces->getNumContactForces(*p_state);
	
	for (ContCount=0; ContCount < NumofCont; ++ContCount) 
	{
     //Get the total spatial force applied to body 2 at the contact point (that is, a force and a moment); 
     // negate this to find the force applied to body 1 at the same point.    	 
	 
	 const ContactForce& force = p_contactForces->getContactForce(*p_state,ContCount);
	 //retrieving information about the contact if we need it
   	 //cout<<"Contact Condition"<<CurContact.getCondition()<<"\n"; // to get Condition of the Contact: 
	 // Unknown - this is an illegal value
	 //Untracked - this pair not yet being tracked; might not contact
     //Anticipated - we expect these to contact soon
     //NewContact - first time seen; needs a ContactId assigned
     //Ongoing 	was new or ongoing before; still in contact now
     //Broken 	was new or ongoing before; no longer in contact 
	 
	  const ContactId     id    = force.getContactId();
      const Vec3 frc = force.getForceOnSurface2()[1];
      const Vec3 mom = force.getForceOnSurface2()[0];
	  const Vec3 AP = force.getContactPoint(); // that are coordinates of the Application Point in fixed coordinate frame

// the following part of code gets information - which bodies of the system are in contact. 
// If a body has two meshes for contact, we should sum the forces on this surfaces.
      Contact CurCont =  p_tracker->getActiveContacts(*p_state).getContactById(id);
	  
	  ContactSurfaceIndex Surf1 = CurCont.getSurface1();
	  ContactSurfaceIndex Surf2 = CurCont.getSurface2();
	  const MobilizedBody Body1 = p_tracker->getMobilizedBody(Surf1);
	  const MobilizedBody Body2 = p_tracker->getMobilizedBody(Surf2);
	  MobilizedBodyIndex Body1Ind = Body1.getMobilizedBodyIndex();
	  MobilizedBodyIndex Body2Ind = Body2.getMobilizedBodyIndex();
	  const Vec3& CP2 = AP - Body2.findBodyOriginLocationInAnotherBody(*p_state,p_matter->updGround());
	  const Vec3& CP1 = AP - Body1.findBodyOriginLocationInAnotherBody(*p_state,p_matter->updGround());

	  // output to Robotran: FonB1 - force on the first body, FonB2 - force on the second body 
      for(i=0;i<NB_CONTACT_BODIES;i++)	
	  {
		  if (Body2Ind == simbodyBodiesStruct->Simbody_index[i])
		  {  
			  simbodyBodiesStruct->force_bodies[i][0]  += frc[0];
			  simbodyBodiesStruct->force_bodies[i][1]  += frc[1];
			  simbodyBodiesStruct->force_bodies[i][2]  += frc[2];
		     
			  simbodyBodiesStruct->torque_bodies[i][0] +=  mom[0] + CP2[1]*frc[2] - CP2[2]*frc[1]; 
	          simbodyBodiesStruct->torque_bodies[i][1] +=  mom[1] + CP2[2]*frc[0] - CP2[0]*frc[2];
	          simbodyBodiesStruct->torque_bodies[i][2] +=  mom[2] + CP2[0]*frc[1] - CP2[1]*frc[0];	  //*/

	  	  }
		  if (Body1Ind == simbodyBodiesStruct->Simbody_index[i])
		  {
		     simbodyBodiesStruct->force_bodies[i][0]  += -frc[0];
			  simbodyBodiesStruct->force_bodies[i][1]  += -frc[1];
			  simbodyBodiesStruct->force_bodies[i][2]  += -frc[2];
		     
			  simbodyBodiesStruct->torque_bodies[i][0] += -mom[0] - CP1[1]*frc[2] + CP1[2]*frc[1];  
	          simbodyBodiesStruct->torque_bodies[i][1] += -mom[1] - CP1[2]*frc[0] + CP1[0]*frc[2];
	          simbodyBodiesStruct->torque_bodies[i][2] += -mom[2] - CP1[0]*frc[1] + CP1[1]*frc[0];//*/
	  	  }
	  }
	}
   return 0;
}

void free_Simbody(void* p_simbodyVariables_void)
{
	SimbodyVariables* p_simbodyVariables = (SimbodyVariables*)p_simbodyVariables_void;

	delete(p_simbodyVariables->p_state); 
    delete(p_simbodyVariables->p_matter);
    delete(p_simbodyVariables->p_tracker);
    delete(p_simbodyVariables->p_contactForces);

	delete(p_simbodyVariables->p_system);
	delete(p_simbodyVariables);
}

/////////////////////////////////////
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

// that part of the code means that there is a ground z = 0;	
	//const Rotation R_zdown(Pi/2.,YAxis);
	//p_matter->Ground().updBody().addContactSurface(
 //       Transform(R_zdown, Vec3(0,0,0)),
 //       ContactSurface(ContactGeometry::HalfSpace(),
 //                      ContactMaterial(k,c,us,ud,uv))); // here we add ground "z+" - is available halfspace.

	// creates a mesh for a ground:
	std::ifstream meshFileGr;
	PolygonalMesh GroundMesh;
		printf("1. Open mesh-file %s ...","ground_mine.obj");
		meshFileGr.open(PROJECT_ABS_PATH"/src/generic/simbody/ground_mine.obj"); 
		printf(" succeed! \n");
		printf("2. Load a mesh from Obj-file ... ");
		GroundMesh.loadObjFile(meshFileGr); 
		printf(" succeed! \n");
	    meshFileGr.close();
				
		printf("3. Transform a mesh ... ");
		//WheelMesh.scaleMesh(0.0254);
		const Rotation R_0(Pi/2.,XAxis);
		//const Rotation R_2(-Pi/2.,ZAxis);
		GroundMesh.transformMesh(Transform(R_0,Vec3(-2.5,-8,-0.1)));
		printf(" succeed! \n");

// apply a contactGeometry for Ground
	printf("4. Convert a polygonal mesh to triangle mesh ... ");

	ContactGeometry::TriangleMesh GroundTrM(GroundMesh); 

	printf(" succeed! ");
	printf("Number of faces = %i ",GroundTrM.getNumFaces());
	printf("5. Add contact surface ... ");
	p_matter->Ground().updBody().addContactSurface(Transform(Vec3(0,0,0)),
							   ContactSurface(GroundTrM,
											   ContactMaterial(k/20,c/100,us,ud,uv),
											   0.001) 
											   );
	printf(" succeed! ");//*/
#ifdef VIZ
	DecorativeMesh showGround = DecorativeMesh(GroundMesh);
	p_matter->updGround().addBodyDecoration(Transform(Vec3(0,0,0)),showGround);
#endif
   
	const Vec3 comLoc(0, 0, 0);  // Location of the center of mass

// set the mass-inertial properties of the body. This parameters might be arbitrary
	const Inertia centralInertia(Vec3(17,2,16), Vec3(0,0,.2)); // 
 	Body::Rigid WheelBody(MassProperties(BoxMass, comLoc, centralInertia));
	
	// creates a mesh for a wheel:
	std::ifstream meshFile1;
	PolygonalMesh WheelMesh;
		printf("1. Open mesh-file %s ...","Leg_AllTerrainRobot.obj");
		meshFile1.open(PROJECT_ABS_PATH"/src/generic/simbody/Leg_AllTerrainRobot.obj"); 
		printf(" succeed! \n");
		printf("2. Load a mesh from Obj-file ... ");
		WheelMesh.loadObjFile(meshFile1); 
		printf(" succeed! \n");
	    meshFile1.close();
				
		printf("3. Transform a mesh ... ");
		//WheelMesh.scaleMesh(0.0254);
		const Rotation R_1(-Pi/2.,YAxis);
		//const Rotation R_2(-Pi/2.,ZAxis);
		WheelMesh.transformMesh(Transform(R_1,Vec3(0,-0.015,-0.032)));
		printf(" succeed! \n");

// apply a contactGeometry for WheelBody
	printf("4. Convert a polygonal mesh to triangle mesh ... ");

	ContactGeometry::TriangleMesh WheelTrM(WheelMesh); 

	printf(" succeed! ");
	printf("Number of faces = %i ",WheelTrM.getNumFaces());
	printf("5. Add contact surface ... ");
	WheelBody.addContactSurface(Transform(Vec3(0,0,0)),
							   ContactSurface(WheelTrM,
											   ContactMaterial(k*10,c*20,us,ud,uv),
											   0.008) // thickness, it is a parameter of the Compliant Contact - thickness of the layer of springs
											   );//*/
	printf(" succeed! ");

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
		
		exit(306081);
	}
 return 0;
}

#endif
