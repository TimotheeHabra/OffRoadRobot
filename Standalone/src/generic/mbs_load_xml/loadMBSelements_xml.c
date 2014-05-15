/*
 * Functions to initialize the MBSdata structure from the <ProjectName>.mbsdata (xml format) file.
 *
 * Allan Barrea Feb. 2013
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include "user_sf_IO.h"
#include "MBSfun.h"
#include "sfdef.h"
#include "MBSdef.h"

/*
 * Code based on loadMBSdata (in MBSdataStruct.c)
 */
void loadMBSelements_xml(MBSdataStruct *s, xmlDocPtr doc, xmlNodePtr cur)
{
    int i,j, ncol;

    //===  Index parameters  ======================================//

	/* qc */
	if (s->nqc)
	{
		s->qc = (int*) calloc(s->nqc+1,sizeof(int));
		s->qc[0] = s->nqc;
		parseVectorInteger_xml("qc", doc, cur, s->qc);
	}
	else
		s->qc = NULL;

	/* qu */
	if (s->nqu)
	{
		s->qu = (int*) calloc(s->nqu+1,sizeof(int));
		s->qu[0] = s->nqu;
		parseVectorInteger_xml("qu", doc, cur, s->qu);
	}
	else
		s->qu = NULL;

	/* qa */
	if (s->nqa)
	{
		s->qa = (int*) calloc(s->nqa+1,sizeof(int));
		s->qa[0] = s->nqa;
		parseVectorInteger_xml("qa", doc, cur, s->qa);
	}
	else
		s->qa = NULL;

	/* qv */
	if (s->nqv)
	{
		s->qv = (int*) calloc(s->nqv+1,sizeof(int));
		s->qv[0] = s->nqv;
		parseVectorInteger_xml("qv", doc, cur, s->qv);
	}
	else
		s->qv = NULL;

	/* hu */ // JFC : 15/01/2008 : ajout
	if (s->nhu)
	{
		s->hu = (int*) calloc(s->nhu+1,sizeof(int));
		s->hu[0] = s->nhu;
		parseVectorInteger_xml("hu", doc, cur, s->hu);
	}
	else
		s->hu = NULL;

	/* qlocked */
	if (s->nqlocked)
	{
		s->qlocked = (int*) calloc(s->nqlocked+1,sizeof(int));
		s->qlocked[0] = s->nqlocked;
		parseVectorInteger_xml("qlocked", doc, cur, s->qlocked);
	}
	else
		s->qlocked = NULL;

	/* qdriven */
	if (s->nqdriven)
	{
		s->qdriven = (int*) calloc(s->nqdriven+1,sizeof(int));
		s->qdriven[0] = s->nqdriven;
		parseVectorInteger_xml("qdriven", doc, cur, s->qdriven);
	}
	else
		s->qdriven = NULL;

////===   Initial values   ==========================================================//
//
//	/* q0,qd0,qdd0 *
//	field_value_ptr  = mxGetField(s_ptr, 0,  "q0");
//	field_value_ptr2 = mxGetField(s_ptr, 0,  "qd0");
//	field_value_ptr3 = mxGetField(s_ptr, 0,  "qdd0");
//	if (field_value_ptr == NULL)
//		mexErrMsgTxt("Could not get field \"q0\"\n");
//	if (field_value_ptr2 == NULL)
//    	mexErrMsgTxt("Could not get field \"qd0\"\n");
//	if (field_value_ptr3 == NULL)
//		mexErrMsgTxt("Could not get field \"qdd0\"\n");
//	s->q0   = (double*) calloc(s->njoint+1,sizeof(double));
//	s->qd0  = (double*) calloc(s->njoint+1,sizeof(double));
//	s->qdd0 = (double*) calloc(s->njoint+1,sizeof(double));
//	s->q0[0]   = (double) s->njoint;
//	s->qd0[0]  = (double) s->njoint;
//	s->qdd0[0] = (double) s->njoint;
//	for(i=1;i<=s->njoint;i++)
//	{
//		s->q0[i]   = mxGetPr(field_value_ptr) [i-1];
//		s->qd0[i]  = mxGetPr(field_value_ptr2)[i-1];
//		s->qdd0[i] = mxGetPr(field_value_ptr3)[i-1];
//	}

//===   Range values   ==========================================================//

	/* qmin */
	if(isElement_xml(cur, "qmin"))
	{
		// vérifier la taille de qmin
		// ...
		s->qmin = (double*) calloc(s->njoint+1,sizeof(double));
		s->qmin[0] = (double) s->njoint;
		// copier les valeurs de qmin
		// ...
	}
	else
		s->qmin = NULL;

	/* qmax */
	if(isElement_xml(cur, "qmax"))
	{
		// vérifier la taille de qmax
		// ...
		s->qmax = (double*) calloc(s->njoint+1,sizeof(double));
		s->qmax[0] = (double) s->njoint;
		// copier les valeurs de qmax
		// ...
	}
	else
		s->qmax = NULL;

//===   Work Variables   ==========================================================//

	/* q,qd,qdd */
	s->q   = (double*) calloc(s->njoint+1,sizeof(double));
	s->qd  = (double*) calloc(s->njoint+1,sizeof(double));
	s->qdd = (double*) calloc(s->njoint+1,sizeof(double));
	s->q[0]   = (double) s->njoint;
	s->qd[0]  = (double) s->njoint;
	s->qdd[0] = (double) s->njoint;
	parseVectorDouble_xml("q", doc, cur, s->q);
	parseVectorDouble_xml("qd", doc, cur, s->qd);
	parseVectorDouble_xml("qdd", doc, cur, s->qdd);

	/* frc */
	s->frc[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->frc[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->frc[i][0] = (double) s->nbody;
		for(j=1;j<=s->nbody;j++)
			s->frc[i][j] = 0.0;
	}

	/* trq */
	s->trq[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->trq[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->trq[i][0] = (double) s->nbody;
		for(j=1;j<=s->nbody;j++)
			s->trq[i][j] = 0.0;
	}

	/* Qq */
	s->Qq = (double*) calloc(s->njoint+1,sizeof(double));
	s->Qq[0] = (double) s->njoint;
	for(i=1;i<=s->njoint;i++)
		s->Qq[i] = 0.0;

	/* tsim */
	s->tsim = 0.0;

//===   System parameters   ====================================================//

	/* dpt */
	s->dpt[0] = NULL;
	if (s->npt)
	{
		for(i=1;i<=3;i++)
		{
			s->dpt[i] = (double*) calloc(s->npt+1,sizeof(double));
			s->dpt[i][0] = (double) s->npt;
		}
		parseMatrixDouble_xml("dpt", doc, cur, s->dpt);
	}
	else // je pense que ce cas est impossible mais il faudrait vérifier avant de supprimer
		for(i=1;i<=3;i++)
			s->dpt[i] = NULL;

	/* l */
	s->l[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->l[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->l[i][0] = (double) s->nbody;
	}
	parseMatrixDouble_xml("l", doc, cur, s->l);

	/* m */
	s->m = (double*) calloc(s->nbody+1,sizeof(double));
	s->m[0] = (double) s->nbody;
	parseVectorDouble_xml("m", doc, cur, s->m);

	/* In */
	s->In[0] = NULL;
	for(i=1;i<=9;i++)
	{
		s->In[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->In[i][0] = (double) s->nbody;
	}
	parseMatrixDouble_xml("In", doc, cur, s->In);

	/* g */
	s->g[0] = 0.0;
    parseVectorDouble_xml("g", doc, cur, s->g);

/* Constraint data */

	/* lrod */
	ncol = getNbColElement_xml("lrod", doc, cur);
	if (ncol)
	{
		s->lrod = (double*) calloc(ncol+1,sizeof(double));
		s->lrod[0] = ncol;
		parseVectorDouble_xml("lrod", doc, cur, s->lrod);
	}
	else
		s->lrod = NULL;

	/* NRerr */
	if (!isElement_xml(cur,"NRerr"))
		s->NRerr = 1e-9; //default value
	else
		s->NRerr = parseScalarInteger_xml("NRerr", doc, cur);

/* Link Data */

	/* Z,Zd,Fl */
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

	/* l3DWr */
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

/* Ext. Forces Data */

	/* xfidpt */
	if (s->Nxfrc)
	{
		s->xfidpt = (int*) calloc(s->Nxfrc+1,sizeof(int));
		s->xfidpt[0] = s->Nxfrc;
        parseVectorInteger_xml("xfidpt", doc, cur, s->xfidpt);
	}
	else
		s->xfidpt = NULL;

	/* SWr */
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

/* Wheel Data */

	/* rnom */
	if (s->Nwheel)
	{
		s->rnom = (double*) calloc(s->Nwheel+1,sizeof(double));
		s->rnom[0] = (double) s->Nwheel;
		parseVectorDouble_xml("rnom", doc, cur, s->rnom);
	}
	else
		s->rnom = NULL;

#if !defined SENSORKIN
/* User Model */

	/* user_model */
	if (s->Nuser_model)
	{
		s->user_model = loadUserModel_xml(doc, cur);
	}

	/* ux,uxd,ux0 */
	if (s->Nux)
	{
		s->ux  = (double*) calloc(s->Nux+1,sizeof(double));
		s->uxd = (double*) calloc(s->Nux+1,sizeof(double));
		s->ux0 = (double*) calloc(s->Nux+1,sizeof(double));
		s->ux[0]  = (double) s->Nux;
		s->uxd[0] = (double) s->Nux;
		s->ux0[0] = (double) s->Nux;
		parseVectorDouble_xml("ux", doc, cur, s->ux);
		parseVectorDouble_xml("uxd", doc, cur, s->uxd);
		parseVectorDouble_xml("ux0", doc, cur, s->ux0);
	}
	else
	{
		s->ux  = NULL;
		s->uxd = NULL;
		s->ux0 = NULL;
	}

//#ifndef CMEX
/* User IO */
	s->user_IO = initUserIO(s);
//#endif

#endif

//===  Other  ========================================================//

	/* qddu */
	if (s->nqu)
	{
		s->qddu = (double*) calloc(s->nqu+1,sizeof(double));
		s->qddu[0] = (double) s->nqu;
	}
	else
		s->qddu = NULL;

	/* DonePart */
	s->DonePart = parseScalarInteger_xml("DonePart", doc, cur);
}
