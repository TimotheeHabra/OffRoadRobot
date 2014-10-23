
#include "mds_translator.h"


 int* MDS_translate_q(int* q_mds ,int nq)
{
	int* q_MBSdataStruct;
	if(nq)
	{
		int i; 
		
		q_MBSdataStruct = (int*) calloc(nq+1,sizeof(int));
		q_MBSdataStruct[0] = nq;
		for(i=0; i<nq; i++)
		{
			q_MBSdataStruct[i+1] =q_mds[i]+1;
		}
	}
	else
	{
		q_MBSdataStruct = NULL;
	}
	return q_MBSdataStruct;
}

MBSdataStruct* MDS_create_MBSdataStruct(MDS_gen_strct* mds_gen_strct)
{
    int i,j;
	int ind_joint = 0;
	int ind_state_value = 0;

    MBSdataStruct *s;

	s = (MBSdataStruct*) malloc(sizeof(MBSdataStruct));

	s->nbody    = mds_gen_strct->bodytree->n_joint;
    s->njoint   = mds_gen_strct->bodytree->n_joint;

	s->npt      = mds_gen_strct->n_point;

	s->nqu      = mds_gen_strct->bodytree->n_qu;
	s->nqv      = mds_gen_strct->bodytree->n_qv;
    s->nqc      = mds_gen_strct->bodytree->n_qc;
	s->nqlocked = mds_gen_strct->bodytree->n_qlocked;
    s->nqdriven = mds_gen_strct->bodytree->n_qdriven;
    s->nqa      = mds_gen_strct->bodytree->n_qa;

    mds_gen_strct->cuts = init_MDS_cuts_strct(); // debug (to be corrected)
    mds_gen_strct->links = init_MDS_links_strct(); // debug (to be corrected)
    
	s->Nloopc   = mds_gen_strct->cuts->n_rod + 3 * mds_gen_strct->cuts->n_ball + 6 * mds_gen_strct->cuts->n_solid;
    s->Nuserc   = 0; 
    s->Ncons    = s->Nloopc + s->Nuserc;

	//s->nhu      = 8; // caution WEMOOV dependent !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	s->nhu      =  s->Ncons;

	s->Nlink    = mds_gen_strct->links->n_link;
    s->Nlink3D  = mds_gen_strct->links->n_link3D;
	s->Nsensor  = mds_gen_strct->n_sensor;
	s->Nxfrc    = mds_gen_strct->n_extforce;

    s->Nwheel   = 0; // dead function (at this time)

#if !defined SENSORKIN
	s->Nuser_model = mds_gen_strct->user_models->n_user_model;
#endif
	s->Nux = mds_gen_strct->n_state_value;


//	geometric ans dynamic data

	//pt

	s->dpt[0] = NULL;
	if (s->npt)
	{
		for(i=1;i<=3;i++)
		{
			s->dpt[i] = (double*) calloc(s->npt+1,sizeof(double));
			s->dpt[i][0] = (double) s->npt;
		}
		for(i=0; i<mds_gen_strct->n_point; i++)
		{
			for(j=0; j<3; j++)
			{
				s->dpt[j+1][i+1] = mds_gen_strct->point_list[i]->pt[j]; 
			}
		}
	}
	else 
		for(i=1;i<=3;i++)
			s->dpt[i] = NULL;

	// l, m, In

	ind_joint = 0;

	s->l[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->l[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->l[i][0] = (double) s->nbody;
	}
	
	s->m = (double*) calloc(s->nbody+1,sizeof(double));
	s->m[0] = (double) s->nbody;

	s->In[0] = NULL;
	for(i=1;i<=9;i++)
	{
		s->In[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->In[i][0] = (double) s->nbody;
	}

	for(i=0; i<mds_gen_strct->bodytree->n_body; i++)
	{
		ind_joint = ind_joint + mds_gen_strct->bodytree->body_list[i]->n_joint;
		for(j=0; j<3; j++)
		{
			s->l[j+1][ind_joint] = mds_gen_strct->bodytree->body_list[i]->com[j]; 
		}

		s->m[ind_joint] = mds_gen_strct->bodytree->body_list[i]->mass;

		s->In[1][ind_joint] = mds_gen_strct->bodytree->body_list[i]->inertia[0];
		s->In[2][ind_joint] = mds_gen_strct->bodytree->body_list[i]->inertia[1];
		s->In[3][ind_joint] = mds_gen_strct->bodytree->body_list[i]->inertia[2]; 
		s->In[4][ind_joint] = mds_gen_strct->bodytree->body_list[i]->inertia[1]; 
		s->In[5][ind_joint] = mds_gen_strct->bodytree->body_list[i]->inertia[3];
		s->In[6][ind_joint] = mds_gen_strct->bodytree->body_list[i]->inertia[4]; 
		s->In[7][ind_joint] = mds_gen_strct->bodytree->body_list[i]->inertia[2]; 
		s->In[8][ind_joint] = mds_gen_strct->bodytree->body_list[i]->inertia[4];
		s->In[9][ind_joint] = mds_gen_strct->bodytree->body_list[i]->inertia[5]; 
	}
	
	// g 
	s->g[0] = 0.0;
	copy_double_vec(mds_gen_strct->base->gravity, &(s->g[1]), 3);

// partitionning infos

	s->qu = MDS_translate_q(mds_gen_strct->bodytree->qu, mds_gen_strct->bodytree->n_qu);
	s->qv = MDS_translate_q(mds_gen_strct->bodytree->qv, mds_gen_strct->bodytree->n_qv);
	s->qc = MDS_translate_q(mds_gen_strct->bodytree->qc, mds_gen_strct->bodytree->n_qc);
	s->qlocked = MDS_translate_q(mds_gen_strct->bodytree->qlocked, mds_gen_strct->bodytree->n_qlocked);
	s->qdriven = MDS_translate_q(mds_gen_strct->bodytree->qdriven, mds_gen_strct->bodytree->n_qdriven);
	s->qa = MDS_translate_q(mds_gen_strct->bodytree->qa, mds_gen_strct->bodytree->n_qa);

	

	if (s->nhu) // to change 
	{
		s->hu = (int*) calloc(s->nhu+1,sizeof(int));
		s->hu[0] = s->nhu;
		for(i=0; i<s->nhu; i++)
		{
			s->hu[i+1] = i+1;
		}
	}
	else
		s->hu = NULL;

//	joints data

	s->q0   = (double*) calloc(s->njoint+1,sizeof(double));
	s->qd0  = (double*) calloc(s->njoint+1,sizeof(double));
	s->qdd0 = (double*) calloc(s->njoint+1,sizeof(double));
	s->q0[0]   = (double) s->njoint;
	s->qd0[0]  = (double) s->njoint;
	s->qdd0[0] = (double) s->njoint;

	s->q   = (double*) calloc(s->njoint+1,sizeof(double));
	s->qd  = (double*) calloc(s->njoint+1,sizeof(double));
	s->qdd = (double*) calloc(s->njoint+1,sizeof(double));


	for(i=0; i<mds_gen_strct->bodytree->n_joint; i++)
	{
		s->q0[i+1] = mds_gen_strct->bodytree->joint_list[i]->q0;
		s->qd0[i+1] = mds_gen_strct->bodytree->joint_list[i]->qd0;
		s->qdd0[i+1] = mds_gen_strct->bodytree->joint_list[i]->qdd0;
	}

	copy_double_vec(s->q0, s->q, s->njoint+1);
	copy_double_vec(s->qd0, s->qd, s->njoint+1);
	copy_double_vec(s->qdd0, s->qdd, s->njoint+1);

	s->qmin = NULL; // dead function (at this time)
	s->qmax = NULL;

// Frc, Trq, Qq, tsim
	
	// frc
	s->frc[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->frc[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->frc[i][0] = (double) s->nbody;
		for(j=1;j<=s->nbody;j++)
			s->frc[i][j] = 0.0;
	}

	// trq
	s->trq[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->trq[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->trq[i][0] = (double) s->nbody;
		for(j=1;j<=s->nbody;j++)
			s->trq[i][j] = 0.0;
	}

	// Qq
	s->Qq = (double*) calloc(s->njoint+1,sizeof(double));
	s->Qq[0] = (double) s->njoint;
	for(i=1;i<=s->njoint;i++)
		s->Qq[i] = 0.0;

	// tsim
	s->tsim = 0.0;

// Constraint data 

	// lrod
	if (mds_gen_strct->cuts->n_rod)
	{
		s->lrod = (double*) calloc(mds_gen_strct->cuts->n_rod+1,sizeof(double));
		s->lrod[0] = mds_gen_strct->cuts->n_rod;
		for(i=0; i<mds_gen_strct->cuts->n_rod; i++)
		{
			s->lrod[i+1] = mds_gen_strct->cuts->rod_list[i]->length;
		}
	}
	else
		s->lrod = NULL;

	// NRerr
		s->NRerr = 1e-9; //default value

// Link Data

	// Z, Zd, Fl 
	if (s->Nlink)
	{
		s->Z  = (double*) calloc(s->Nlink+1,sizeof(double));
		s->Zd = (double*) calloc(s->Nlink+1,sizeof(double));
		s->Fl = (double*) calloc(s->Nlink+1,sizeof(double));
		s->Z[0]  = (double) s->Nlink;
		s->Zd[0] = (double) s->Nlink;
		s->Fl[0] = (double) s->Nlink;
		for(i=1;i<=s->Nlink;i++)
		{
			s->Z[i]  = 0.0;
			s->Zd[i] = 0.0;
			s->Fl[i] = 0.0;
		}
	}
	else
	{
		s->Z  = NULL;
		s->Zd = NULL;
		s->Fl = NULL;
	}

	// l3DWr 
	if (s->Nlink3D)
	{
		s->l3DWr = (double**) calloc(s->Nlink3D+1,sizeof(double*));
		s->l3DWr[0] = NULL;
		for(i=1;i<=s->Nlink3D;i++)
		{
			s->l3DWr[i] = (double*) calloc(6+1,sizeof(double));
			for(j=0;j<=6;j++)
				s->l3DWr[i][j] = 0.0;
		}
	}
	else
		s->l3DWr = NULL;

// Ext. Forces Data 

	// xfidpt 
	if (s->Nxfrc)
	{
		s->xfidpt = (int*) calloc(s->Nxfrc+1,sizeof(int));
		s->xfidpt[0] = s->Nxfrc;
		for(i=0;i<mds_gen_strct->n_extforce;i++)
		{
			s->xfidpt[i+1] = mds_gen_strct->extforce_point_list[i]+1;
		}
	}
	else
		s->xfidpt = NULL;

	// SWr
	if (s->Nxfrc)
	{
		s->SWr = (double**) calloc(s->Nxfrc+1,sizeof(double*));
		s->SWr[0] = NULL;
		for(i=1;i<=s->Nxfrc;i++)
		{
			s->SWr[i] = (double*) calloc(9+1,sizeof(double));
			for(j=0;j<=9;j++)
				s->SWr[i][j] = 0.0;
		}
	}
	else
		s->SWr = NULL;

// Wheel Data 

	// rnom
	if (s->Nwheel)
	{
		s->rnom = (double*) calloc(s->Nwheel+1,sizeof(double));
		s->rnom[0] = (double) s->Nwheel;
		// to complet
	}
	else
		s->rnom = NULL;

#if !defined SENSORKIN
// User Model

	// user_model
	if (s->Nuser_model)
	{
		s->user_model = init_UserModelStruct();
		load_UserModelStruct(mds_gen_strct, s->user_model);
	}

	// ux, uxd, ux0
	if (s->Nux)
	{
		s->ux  = (double*) calloc(s->Nux+1,sizeof(double));
		s->uxd = (double*) calloc(s->Nux+1,sizeof(double));
		s->ux0 = (double*) calloc(s->Nux+1,sizeof(double));
		s->ux[0]  = (double) s->Nux;
		s->uxd[0] = (double) s->Nux;
		s->ux0[0] = (double) s->Nux;
		
		for(i=0; i<mds_gen_strct->n_state; i++)
		{
			for(j=0; j<mds_gen_strct->state_list[i]->n_value; j++)
			{
				s->ux0[ind_state_value+1] = mds_gen_strct->state_list[i]->value_list[j];
				ind_state_value++;
			}
		}
		copy_double_vec(s->ux0, s->ux, s->Nux);
	}
	else
	{
		s->ux  = NULL;
		s->uxd = NULL;
		s->ux0 = NULL;
	}

	s->user_IO = initUserIO(s);

#endif

//	Other

	// qddu
	if (s->nqu)
	{
		s->qddu = (double*) calloc(s->nqu+1,sizeof(double));
		s->qddu[0] = (double) s->nqu;
	}
	else
		s->qddu = NULL;


	s->DonePart = 0;
	s->DoneEquil = 0;

	return s;
}


////////////////////////////////////////////////////////////////////////////        CAUTION  CAUTION 


void MDS_free_MBSdataStruct(MBSdataStruct *s)
{
	int i;

	// Donnees geometriques et dynamiques
	if (s->npt)
		for(i=1;i<=3;i++)
			free(s->dpt[i]);

	for(i=1;i<=3;i++)
		free(s->l[i]);

	free(s->m);

   	for(i=1;i<=9;i++)
		free(s->In[i]);

	// Infos partitionnement
	if (s->nqlocked)
		free(s->qlocked);
	if (s->nqdriven)
		free(s->qdriven);
	if (s->nqc)
		free(s->qc);
	if (s->nqu)
		free(s->qu);
	if (s->nqa)
		free(s->qa);
	if (s->nqv)
		free(s->qv);

	// Variables articulaires, valeures initiales, limites
	free(s->q);
	free(s->qd);
	free(s->qdd);
	free(s->q0);
	free(s->qd0);
	free(s->qdd0);


	if (s->qmin != NULL)
		free(s->qmin);
	if (s->qmax != NULL)
		free(s->qmax);

	// frc, trq, Qq
   	for(i=1;i<=3;i++)
	{
		free(s->frc[i]);
		free(s->trq[i]);
	}
	free(s->Qq);

	// Constraints
	if (s->lrod != NULL)
		free(s->lrod);

	// Links
	if (s->Nlink)
	{
		free(s->Z);
		free(s->Zd);
		free(s->Fl);
	}
	if (s->Nlink3D)
		for(i=1;i<=6;i++)
			free(s->l3DWr[i]);

	// Ext. forces
	if (s->Nxfrc)
	{
		free(s->xfidpt);
		for(i=1;i<=s->Nxfrc;i++)
			free(s->SWr[i]);
		free(s->SWr);
	}

	// Wheel
	if (s->Nwheel)
	{
		free(s->rnom);
	}

	// User state
	if (s->Nux)
	{
		free(s->ux);
		free(s->uxd);
		free(s->ux0);    
	}

#ifndef SENSORKIN
	// User models
	free_UserModelStruct(s->user_model);
	freeUserIO(s->user_IO,s);
#endif

	// Other
	if (s->nqu)
	{
		free(s->qddu);
	}

	free(s);

}