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
	
	// creates new instances for internal Simbody system variables
    p_simbodyVariables->p_system        = new MultibodySystem;
    p_simbodyVariables->p_matter        = new SimbodyMatterSubsystem(*(p_simbodyVariables->p_system));
    p_simbodyVariables->p_tracker       = new ContactTrackerSubsystem(*(p_simbodyVariables->p_system));
    p_simbodyVariables->p_contactForces = new CompliantContactSubsystem(*(p_simbodyVariables->p_system),*(p_simbodyVariables->p_tracker));

	#ifdef SIMBODYVIZ	
		p_simbodyVariables->p_system->setUpDirection(+ZAxis); // that is for visualization only. The default direction is +X
		p_simbodyVariables->p_viz = new Visualizer(*(p_simbodyVariables->p_system));
	#endif

    init_Simbody(p_simbodyVariables, simbodyBodies);
    


    //it is "system" commands. We cannot avoid them.    
    
	p_simbodyVariables->p_system->realizeTopology();

    p_simbodyVariables->p_state = new State (p_simbodyVariables->p_system->getDefaultState());
	
    //it is "system" command. We cannot avoid them. 
    p_simbodyVariables->p_system->realizeModel(*p_simbodyVariables->p_state);

	return (void*) p_simbodyVariables;
}

int loop_Simbody (SimbodyStruct *simbodyStruct)
{	   
	try
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
	#ifdef SIMBODYVIZ		
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
	  const Vec3 AP = force.getContactPoint(); // that are coordinates of the Application Point in fixed coordinate frame.

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

	  // output to Robotran
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

////////////////////////////////////
//function used by C++ code only
/////////////////////////////////////

int init_Simbody(SimbodyVariables *p_simbodyVariables, SimbodyBodiesStruct *p_simbodyBodiesStruct)
{
try
  { 
	
	MultibodySystem *p_system = p_simbodyVariables->p_system;
	SimbodyMatterSubsystem *p_matter = p_simbodyVariables->p_matter;
	State *p_state = p_simbodyVariables->p_state;

	ContactPropertiesStruct* GrContProp;
	ContactPropertiesStruct* CurBodyContProp;

	GrContProp = &(p_simbodyBodiesStruct->GroundContProp);
	
	if (GrContProp->Geometry == 1)
	{
	// creates a mesh for a ground:
		std::ifstream meshFileGr;
		PolygonalMesh GroundMesh;
		char FileName[FILENAME_MAX];
		printf("1. Open mesh-file %s ...",GrContProp->FileName);
		sprintf(FileName,"%s%s%s",PROJECT_ABS_PATH,"/src/project/simbody/",GrContProp->FileName);	
		meshFileGr.open(FileName); 

		printf(" succeed! \n");
		printf("2. Load a mesh from Obj-file ... ");
		GroundMesh.loadObjFile(meshFileGr); 
		printf(" succeed! \n");
	    meshFileGr.close();
				
		printf("3. Transform a mesh ... ");
		GroundMesh.scaleMesh(GrContProp->ScaleFactor);
		const Rotation R_x(GrContProp->Rotation[0]*Pi/180.,XAxis);
		const Rotation R_y(GrContProp->Rotation[1]*Pi/180.,YAxis);
		const Rotation R_z(GrContProp->Rotation[2]*Pi/180.,ZAxis);

		GroundMesh.transformMesh(Transform(R_x*R_y*R_z,Vec3(GrContProp->Transform))); //TODO check the order of rotations!
		printf(" succeed! \n");

// apply a contactGeometry for Ground
	printf("4. Convert a polygonal mesh to triangle mesh ... ");

	ContactGeometry::TriangleMesh GroundTrM(GroundMesh); 

	printf(" succeed! \n");
	printf("Number of faces = %i\n",GroundTrM.getNumFaces());
	printf("5. Add contact surface ... ");
	p_matter->Ground().updBody().addContactSurface(Transform(Vec3(0,0,0)),
							   ContactSurface(GroundTrM,
											   ContactMaterial(GrContProp->k,GrContProp->c,GrContProp->us,GrContProp->ud,GrContProp->uv),
											   GrContProp->thickness) 
											   );
	printf(" succeed! \n\n");//*/
#ifdef SIMBODYVIZ
	p_simbodyVariables->p_viz =&( p_simbodyVariables->p_viz->setBackgroundType(Visualizer::BackgroundType(0)));
	DecorativeMesh showGround = DecorativeMesh(GroundMesh);
	p_matter->updGround().addBodyDecoration(Transform(), showGround.setColor(Blue).setOpacity(1).setRepresentation(SimTK::DecorativeGeometry::Representation(2)));
	p_matter->updGround().addBodyDecoration(Transform(), showGround.setColor(Gray).setOpacity(1).setRepresentation(SimTK::DecorativeGeometry::Representation(0)));
#endif
	}
	else
	{
		#ifdef SIMBODYVIZ
		p_simbodyVariables->p_viz =&(p_simbodyVariables->p_viz->setBackgroundType(Visualizer::BackgroundType(1)));
		#endif
		const Rotation R_zdown(Pi/2.,YAxis);
		p_matter->Ground().updBody().addContactSurface(Transform(R_zdown, Vec3(0,0,0)),
        ContactSurface(ContactGeometry::HalfSpace(),
                       ContactMaterial(GrContProp->k,GrContProp->c,GrContProp->us,GrContProp->ud,GrContProp->uv)));
	}
	

   
	const Vec3 comLoc(0, 0, 0);  // Location of the center of mass
// set the mass-inertial properties of the body. These parameters might be arbitrary
	const Inertia centralInertia(Vec3(17,2,16), Vec3(0,0,.2)); // 
	const Real BoxMass = 1.0;// kg -> not used

	int i;
	for (i=0;i<p_simbodyBodiesStruct->nb_contact_bodies;i++)
	{
	// creates a mesh for a wheel:
		
		Body::Rigid Body(MassProperties(BoxMass, comLoc, centralInertia));
		CurBodyContProp = &(p_simbodyBodiesStruct->BodyContProp[i]);
		char BodyFileName[FILENAME_MAX];
		std::ifstream meshFile1;
	    PolygonalMesh BodyMesh;
		
		printf("1. Open mesh-file %s ...",CurBodyContProp->FileName);
	    sprintf(BodyFileName,"%s%s%s",PROJECT_ABS_PATH,"/src/project/simbody/",CurBodyContProp->FileName);	
	    meshFile1.open(BodyFileName); 
		
		printf(" succeed! \n");
		printf("2. Load a mesh from Obj-file ... ");
		BodyMesh.loadObjFile(meshFile1); 
		printf(" succeed! \n");
	    meshFile1.close();
				
		printf("3. Transform a mesh ... ");

		BodyMesh.scaleMesh(CurBodyContProp->ScaleFactor);
		const Rotation R_1(CurBodyContProp->Rotation[0]*Pi/180.,XAxis);
		const Rotation R_2(CurBodyContProp->Rotation[1]*Pi/180.,YAxis);
		const Rotation R_3(CurBodyContProp->Rotation[2]*Pi/180.,ZAxis);

		BodyMesh.transformMesh(Transform(R_1*R_2*R_3,Vec3(CurBodyContProp->Transform))); 
		printf(" succeed! \n");
				

    // apply a contactGeometry for WheelBody
	printf("4. Convert a polygonal mesh to triangle mesh ... ");

	ContactGeometry::TriangleMesh BodyTrM(BodyMesh); 

	printf(" succeed! \n");
	printf("Number of faces = %i\n",BodyTrM.getNumFaces());
	printf("5. Add contact surface ... ");
	Body.addContactSurface(Transform(Vec3(0,0,0)),
							   ContactSurface(BodyTrM,
											   ContactMaterial(CurBodyContProp->k,CurBodyContProp->c,CurBodyContProp->us,CurBodyContProp->ud,CurBodyContProp->uv),
											   CurBodyContProp->thickness) 
											   );
	printf(" succeed! \n\n");

// it is only for visualization. Doesn't work with Linux
 	#ifdef SIMBODYVIZ
		DecorativeMesh showBox = DecorativeMesh(BodyMesh);
		Body.addDecoration(Transform(), showBox.setColor(Red).setOpacity(1).setRepresentation(SimTK::DecorativeGeometry::Representation(0)));
    #endif // VIZ

	MobilizedBody::Free MobBody(p_matter->Ground(), Transform(Vec3(0)),
        Body, Transform(Vec3(0)));
    	p_simbodyBodiesStruct->Simbody_index[i] = MobBody.getMobilizedBodyIndex(); 
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
