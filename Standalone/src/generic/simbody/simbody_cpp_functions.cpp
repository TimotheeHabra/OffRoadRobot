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

	

    init_Simbody(p_simbodyVariables, simbodyBodies);
    
	#ifdef VIZ	
	p_simbodyVariables->p_system->setUpDirection(+ZAxis); // that is for visualization only. The default direction is +X
	Visualizer viz(system);
	#endif

    //it is "system" commands. We cannot avoid them.    
    
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


#endif
