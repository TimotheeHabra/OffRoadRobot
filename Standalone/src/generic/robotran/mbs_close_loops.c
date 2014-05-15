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
// 01/10/2008 : JFC : Bug n°40
//

#ifndef ACCELRED

#include "MBSfun.h"
#include "nrfct.h"

double norm_vector(double *v, int n);
double norminf_vector(double *v, int n);

int mbs_close_geo(MBSdataStruct *s, LocalDataStruct *lds)
{
	int i,j;
	int iter=0;
	int nL,nC;

	double d;

	iter = 0;
	lds->norm_h=1.0;
	while((lds->norm_h > lds->NRerr) && (iter++ <= lds->MAX_NR_ITER))
	{
		// Calcul des contraintes et de la Jacobienne
		mbs_calc_hJ(lds,s,s->tsim);

		// Norme des contraintes (en supposant que toutes les contraintes indépendantes sont au début ???)
		lds->norm_h = norminf_vector(lds->h,s->nhu);

		// -Jv
		nL = s->nhu;
		nC = s->nqv;
		for(i=1;i<=nL;i++)
		{
			for(j=1;j<=nC;j++)
			{
				lds->mJv[i][j] = -lds->Jac[s->hu[i]][s->qv[j]];
			}
		}

		// Décomposition LU de la matrice -Jv
		ludcmp(lds->mJv,s->nqv,lds->ind_mJv,&d);

		if(lds->norm_h > lds->NRerr)
		{
			// err 
			for(i=1;i<=s->nhu;i++)
			{
				lds->mJv_h[i] = lds->h[s->hu[i]];
//				lds->mJv_h[i][1] = lds->h[s->hu[i]];
			}
			lubksb(lds->mJv,s->nqv,lds->ind_mJv,lds->mJv_h);
//			gaussj(lds->mJv, s->nqv, lds->mJv_h, 1);

			// Correction des qv
			for(i=1;i<=s->nhu;i++)
			{
				s->q[s->qv[i]] += lds->mJv_h[i];
//				s->q[s->qv[i]] += lds->mJv_h[i][1];
			}
		}
	}
	return iter;
}

void mbs_close_kin(MBSdataStruct *s, LocalDataStruct *lds)
{		
	int i,j,k;
	int nL,nC,nk;

	double term;

	nL = s->nhu;
	nC = lds->iquc[0];
	for(i=1;i<=nL;i++)
	{
		for(j=1;j<=nC;j++)
		{
			lds->Juct[j][i] = lds->Jac[s->hu[i]][lds->iquc[j]];
//			lds->Juc[i][j] = lds->Jac[s->hu[i]][lds->iquc[j]];
		}
	}

	// calcul de la matrice de couplage des vitesses
	for (j=1;j<=nC;j++)
	{
		lubksb(lds->mJv,s->nqv,lds->ind_mJv,lds->Juct[j]);
	}
//	gaussj(lds->mJv, s->nqv, lds->Juc, nC);
	for(i=1;i<=nL;i++)
	{
		for(j=1;j<=nC;j++)
		{
			lds->Bvuc[i][j] = lds->Juct[j][i];
//			lds->Bvuc[i][j] = lds->Juc[i][j];
		}
	}

	// calcul des vitesses dépendantes (qdv = Bvuc * qduc)
	nL = s->nqv;
	nk = lds->iquc[0];
	for(i=1;i<=nL;i++)
	{
		term = 0.0;
		for(k=1;k<=nk;k++)
		{
			term += lds->Bvuc[i][k]*s->qd[lds->iquc[k]];
		}
		s->qd[s->qv[i]] = term;
	}

	// bp = (-Jv)\jdqd
	cons_jdqd(lds->jdqd,s,s->tsim);
	for(i=1;i<=s->nhu;i++)
	{
		lds->bp[i] = lds->jdqd[s->hu[i]];
//		lds->bp[i][1] = lds->jdqd[s->hu[i]];
	}
	lubksb(lds->mJv,s->nqv,lds->ind_mJv,lds->bp);
//	gaussj(lds->mJv, s->nqv, lds->bp, 1);

}

void mbs_calc_hJ(LocalDataStruct *lds, MBSdataStruct *s, double tsim)
{
	int i,j;

	// contraintes de fermeture de boucles
	cons_hJ(lds->h,lds->Jac,s,s->tsim);

	// contraintes user
	if (s->Nuserc>0) {
		user_cons_hJ(lds->huserc, lds->Juserc, s, tsim);

		// ajout des contraintes user aux contraintes de fermeture
		for (i=1;i<=s->Nuserc;i++) {
			lds->h[s->Nloopc+i] = lds->huserc[i];
			for (j=1;j<=s->njoint;j++)
				lds->Jac[s->Nloopc+i][j] = lds->Juserc[i][j];
		}
	}
}

void mbs_calc_jdqd(LocalDataStruct *lds, MBSdataStruct *s, double tsim)
{
	int i;

	// contraintes user
	if (s->Nuserc>0) {
		user_cons_jdqd(lds->jdqduserc, s, tsim);

		// ajout des contraintes user aux contraintes de fermeture
		for (i=1;i<=s->Nuserc;i++) {
			lds->jdqd[s->Nloopc+i] = lds->jdqduserc[i];
		}
	}
}

#endif
