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
 * Frees the memory allocated to the MBSdata structure.
 */
void freeMBSdata_xml(MBSdataStruct *s)
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
//	free(s->q0);
//	free(s->qd0);
//	free(s->qdd0);
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
		free(s->rnom);

	// User state
	if (s->Nux)
	{
		free(s->ux);
		free(s->uxd);		// MD 21/12/2006: probleme inconnu - JFC: y a pas de raison
		free(s->ux0);
	}

#ifndef SENSORKIN
	// User models
	freeUserModel_xml(s->user_model);
//#ifndef CMEX
	// User IO
	freeUserIO(s->user_IO,s);
//#endif
#endif

	// Other
	if (s->nqu)
		free(s->qddu);

	free(s);
}
