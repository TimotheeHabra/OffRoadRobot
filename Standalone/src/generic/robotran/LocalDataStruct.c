//-------------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2008 by JF Collard
// Last update : 01/10/2008
//-------------------------------
//
// Gestion via Bugzilla :
// 01/10/2008 : JFC : Bug n°39
//

#ifndef ACCELRED

#include "MBSfun.h"

#include "mbs_tool.h"
#include "nrutil.h"

#ifndef CMEX
LocalDataStruct * initLocalDataStruct(SimStruct *S, MBSdataStruct *s)
#else
LocalDataStruct * initLocalDataStruct(MBSdataStruct *s)
#endif
{
	LocalDataStruct *lds = (LocalDataStruct*) malloc(sizeof(LocalDataStruct));
	int i;

	#ifdef SENSORKIN
	int j;
	#endif

#if defined DIRDYNARED || defined INVDYNARED
	int njoint, nqu, nqv, nqc, nquc, Ncons, Nuserc;

	#ifdef INVDYNARED
	int nqa;
	#endif

	njoint = s->njoint;
	nqu = s->nqu;
	nqv = s->nqv;
	nqc = s->nqc;

	#ifdef INVDYNARED
	nqa = s->nqa;
	#endif

	nquc = nqu + nqc;
	Nuserc = s->Nuserc;
	Ncons = s->nhu;// JFC : 15/01/2008 : nhu au lieu de Ncons
	// rem: nhu contient les contraintes user

	lds->NRerr = s->NRerr;
	lds->MAX_NR_ITER = 100;
	
	if (Ncons>0)
	{
		lds->h = mbs_vector(Ncons);
		lds->Jac = mbs_matrix(Ncons,njoint);
		lds->bp = mbs_vector(Ncons);
//		lds->bp = mbs_matrix(Ncons,1);
		lds->mJv = mbs_matrix(Ncons,nqv);
		lds->ind_mJv = mbs_ivector(Ncons);
		lds->mJv_h = mbs_vector(nqv);
//		lds->mJv_h = mbs_matrix(nqv,1);

#ifdef MATLAB_MEX_FILE
		if (nquc != njoint - Ncons)
			mexErrMsgTxt("nquc != njoint-Ncons");
#endif

		lds->Juct = mbs_matrix(nquc,Ncons); // inversion des lignes et des colonnes
//		lds->Juc = mbs_matrix(Ncons,nquc);
		lds->Bvuc = mbs_matrix(nqv,nquc);
		lds->jdqd = mbs_vector(Ncons);
	}
	else
	{
		lds->h = NULL;
		lds->Jac = NULL;
		lds->bp = NULL;
		lds->mJv = NULL;
		lds->ind_mJv = NULL;
		lds->mJv_h = NULL;
		lds->Juct = NULL;
		lds->Bvuc = NULL;
		lds->jdqd = NULL;
	}

	if (Nuserc>0)
	{
		lds->huserc = mbs_vector(Nuserc);
		lds->Juserc = mbs_matrix(Nuserc,njoint);
		lds->jdqduserc = mbs_vector(Nuserc);
	}
	else
	{
		lds->huserc = NULL;
		lds->Juserc = NULL;
		lds->jdqduserc = NULL;
	}

	// 11/1/2008 JFC : modification
	if (nquc)
	{
		lds->iquc = mbs_ivector(nquc);
		lds->iquc[0]=nquc;
		for(i=1;i<=nqu;i++)
			lds->iquc[i] = s->qu[i];
		for(i=1;i<=nqc;i++)
			lds->iquc[i+nqu] = s->qc[i];
	}
	else
		lds->iquc = NULL;

	allocate_sensor(lds->psens, njoint);
	init_sensor(lds->psens, njoint);

#elif defined SENSORKIN
#ifndef CMEX
	const int nsensor = mxGetNumberOfElements(ssGetSFcnParam(S,2)); // get 3rd S-function parameter size
#endif
	if (nsensor) {
		lds->psensorStruct = (MBSsensorStruct**) calloc(nsensor+1,sizeof(MBSsensorStruct*));
		lds->psensorStruct[0] = NULL;
		for (i=1;i<=nsensor;i++) {
			lds->psensorStruct[i] = (MBSsensorStruct*) malloc(sizeof(MBSsensorStruct));
			//double *J[7]; //attention: nécessite allocation dynamique en J[7][njoint+1]
			lds->psensorStruct[i]->J[0] = NULL;
			for (j=1;j<=6;j++) {
				lds->psensorStruct[i]->J[j] = (double*) calloc(s->njoint+1,sizeof(double));
				lds->psensorStruct[i]->J[j][0] = s->njoint;
			}
		}
	}
	else
		lds->psensorStruct = NULL;
#endif

#ifdef DIRDYNARED
	lds->M = mbs_matrix(njoint,njoint);
	lds->c = mbs_vector(njoint);
	lds->F = mbs_vector(njoint);

	if (Ncons>0)
	{
		lds->BtMvu = mbs_matrix(nquc,nquc);
		lds->BtMvv = mbs_matrix(nquc,nqv);
		lds->BtMB = mbs_matrix(nquc,nquc);

		lds->BtFv = mbs_vector(nquc);
		lds->MBMb = mbs_vector(nquc); // JFC 4/02/2008 : correction de nqu en nquc
	}

	lds->Mruc = mbs_matrix(nquc,nquc); // alloué désalloué dans le code : y a pas de raison ?
	lds->Fruc = mbs_vector(nquc); // alloué désalloué dans le code : y a pas de raison ?

	lds->Mr = mbs_matrix(nqu,nqu); // alloué désalloué dans le code : idem
	lds->Fr = mbs_vector(nqu);

	lds->p_Mr = mbs_vector(nqu);
//	lds->p_Mr = mbs_ivector(nqu);
	lds->Qc = mbs_vector(nqc);

#elif defined INVDYNARED
/*
	if (Ncons>0)
	{
		lds->lambda = mbs_vector(Ncons);
		lds->mJvt = mbs_matrix(nqv,Ncons);
		lds->ind_mJvt = mbs_ivector(Ncons);
	}
	else
	{
		lds->lambda = NULL;
		lds->mJvt = NULL;
		lds->ind_mJvt = NULL;
	}
*/
	lds->phi = mbs_vector(njoint);
	lds->Qact = mbs_vector(nqa);
	lds->Qc = mbs_vector(nqc);

	lds->A = mbs_matrix(nqu,nqa);
	lds->ind_A = mbs_ivector(nqa);
	lds->b = mbs_vector(nqu);

	lds->w = mbs_vector(nqa);
	lds->v = mbs_matrix(nqa,nqa);
#endif

#ifdef CMEX
	lds->y = dvector(1,2*nqu);
	lds->dydx = dvector(1,2*nqu);

	for(i=1;i<=s->nqu;i++) 
	{ 
		lds->y[i-1] = s->q[s->qu[i]]; 
		lds->y[i+s->nqu-1] = s->qd[s->qu[i]]; 
		lds->dydx[i-1] = s->qd[s->qu[i]]; 
		lds->dydx[i+s->nqu-1] = s->qddu[i]; 
	} 
#endif
	/**/
	return lds;
}

/******************************************************************************/
#ifndef CMEX
void freeLocalDataStruct(SimStruct *S, LocalDataStruct *lds, MBSdataStruct *s)
#else
void freeLocalDataStruct(LocalDataStruct *lds, MBSdataStruct *s)
#endif
{

#if defined DIRDYNARED || defined INVDYNARED
	int njoint, nqu, nqv, nqc, nquc, Ncons, Nuserc;

	#ifdef INVDYNARED
	int nqa;
	#endif

	njoint = s->njoint;
	nqu = s->nqu;
	nqv = s->nqv;
	nqc = s->nqc;

	#ifdef INVDYNARED
	nqa = s->nqa;
	#endif
	
	nquc = nqu + nqc;
	Nuserc = s->Nuserc;
	Ncons = s->nhu;
	/**/

	if (Ncons>0)
	{
		free_mbs_vector(lds->h);
		free_mbs_matrix(lds->Jac,Ncons);
		free_mbs_matrix(lds->mJv,Ncons);
		free_mbs_ivector(lds->ind_mJv);
		free_mbs_vector(lds->mJv_h);
//		free_mbs_matrix(lds->mJv_h,nqv);
		free_mbs_ivector(lds->iquc);
		free_mbs_matrix(lds->Juct,nquc);
//		free_mbs_matrix(lds->Juc,Ncons);
		free_mbs_matrix(lds->Bvuc,nqv);
		free_mbs_vector(lds->jdqd);
		free_mbs_vector(lds->bp);
//		free_mbs_matrix(lds->bp,Ncons);
	}
	if (Nuserc>0)
	{
		free_mbs_vector(lds->huserc);
		free_mbs_matrix(lds->Juserc,Nuserc);
		free_mbs_vector(lds->jdqduserc);
	}

	free_sensor(lds->psens);

#elif defined SENSORKIN
#ifndef CMEX
	const int nsensor = mxGetNumberOfElements(ssGetSFcnParam(S,2)); // get 3rd S-function parameter size
#endif
	int i,j;

	if (nsensor) {
		for (i=1;i<=nsensor;i++)
			for (j=1;j<=6;j++)
				free(lds->psensorStruct[i]->J[j]);
			free(lds->psensorStruct[i]);
	}

	free(lds->psensorStruct);
#endif

#ifdef DIRDYNARED
	free_mbs_matrix(lds->M,njoint);
	free_mbs_vector(lds->c);
	free_mbs_vector(lds->F);

	if (Ncons>0)
	{
		free_mbs_matrix(lds->BtMvu,nquc);
		free_mbs_matrix(lds->BtMvv,nquc);
		free_mbs_matrix(lds->BtMB,nquc);

		free_mbs_vector(lds->BtFv);
		free_mbs_vector(lds->MBMb);
	}
	
	free_mbs_matrix(lds->Mruc,nquc);
	free_mbs_vector(lds->Fruc);
	
	free_mbs_matrix(lds->Mr,nqu);
	free_mbs_vector(lds->Fr);

	free_mbs_vector(lds->p_Mr);
//	free_mbs_ivector(lds->p_Mr);
	free_mbs_vector(lds->Qc);
#endif

#ifdef INVDYNARED
/*
	if (Ncons>0)
	{
		free_mbs_vector(lds->lambda);
		free_mbs_matrix(lds->mJvt,Ncons);
		free_mbs_ivector(lds->ind_mJvt);
	}
*/
	free_mbs_vector(lds->phi);
	free_mbs_vector(lds->Qact);
	free_mbs_vector(lds->Qc);

	free_mbs_matrix(lds->A,nqu);
	free_mbs_vector(lds->b);
	
	free_mbs_vector(lds->w);
	free_mbs_matrix(lds->v,nqa);
#endif

#ifdef CMEX
	free_vector(lds->y,1,2*nqu);
	free_vector(lds->dydx,1,2*nqu);
#endif
/**/
	free(lds);
}

#endif
