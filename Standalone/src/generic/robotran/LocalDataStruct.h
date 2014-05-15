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

#ifndef LocalDataStruct_h
#define LocalDataStruct_h
/*--------------------*/

#ifndef ACCELRED

#include "MBSsensorStruct.h"


 
typedef struct LocalDataStruct_tag
{
#if defined DIRDYNARED || defined INVDYNARED
		double norm_h;
		double NRerr;
		int MAX_NR_ITER;

		double *h, **Jac;
		double *huserc, **Juserc;
		double **mJv;
		int *ind_mJv;

		double *mJv_h;
//		double **mJv_h;
		
		int *iquc;
		double **Juct; // JFC: 15/01/2008 : changement nom
//		double **Juc; // JFC: ajout temporaire

		double **Bvuc;


		double *jdqd;
		double *jdqduserc;
		
		double * bp;
//		double **bp;

		MBSsensorStruct psens[1]; // temporary sensor pointer

#elif defined SENSORKIN
	MBSsensorStruct **psensorStruct;
#endif
/**/

#ifdef DIRDYNARED
		double *c, **M;
		double *F;

		double **BtMvu,**BtMvv,**BtMB;
		double *BtFv,*MBMb;

		double **Mruc;
		double *Fruc;

		double **Mr;
		double *Fr;

		double *p_Mr;
//		int *p_Mr;
		double *Qc;
#endif

#ifdef INVDYNARED
		double *phi;
		double *Qact;
		double *Qc;

		double **A;
		int *ind_A;
		double *b;

		double *w;
		double **v;

//		double **mJvt;
//		int    *ind_mJvt;
//		double *lambda;
#endif

#ifdef CMEX
		double x, *y, *dydx;
#endif
} LocalDataStruct;

#else
typedef void* LocalDataStruct;

#endif
#endif
