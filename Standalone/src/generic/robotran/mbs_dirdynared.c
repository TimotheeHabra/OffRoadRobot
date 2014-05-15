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
// 01/10/2008 : JFC : Bug n°41
//

#ifdef DIRDYNARED

#include "MBSfun.h"
#include "math.h"
#include "nrfct.h"

int dirdynared(LocalDataStruct *lds,MBSdataStruct *s)
{
	int i,j,k;
	int iter=0;
	int nL,nC,nk;

	double term;

	int has_a_line_of_zeros;

	// Expression des variables commandées
	if (s->nqc>0) user_DrivenJoints(s,s->tsim);

	// Résolution des Contraintes
	if (s->Ncons > 0)
	{
		// résolution géométrique
		iter = mbs_close_geo(s, lds);

		if (iter>=lds->MAX_NR_ITER)
		{
			return -1;
		}

		// résolution cinématique
		mbs_close_kin(s, lds);

	}//if(s->Ncons > 0)

	// calcul des forces appliquées sur les corps
	for(i=1;i<=s->nbody;i++)
	{
		for(j=1;j<=3;j++)
		{
			s->frc[j][i]=0.0;
			s->trq[j][i]=0.0;
		}
	}

	#ifdef CXX
	if(s->Nlink > 0) link1D(s->frc,s->trq,s->Fl,s->Z,s->Zd,s,s->tsim);
	#else
	if(s->Nlink > 0) link(s->frc,s->trq,s->Fl,s->Z,s->Zd,s,s->tsim);
	#endif

	if(s->Nxfrc > 0) extforces(s->frc,s->trq,s,s->tsim);

	s->Qq = user_JointForces(s,s->tsim);


	// calcul de la matrice de masse et du vecteur des forces non linéaires
	dirdyna(lds->M,lds->c,s,s->tsim);

	for(i=1;i<=s->njoint;i++)
	{
		lds->F[i] = lds->c[i] - s->Qq[i];
	}

	// calcul de la matrice de masse et du vecteur des forces reduites (DAE => ODE)
	if (s->nqv>0)
	{
		//	Mr_uc = Muc_uc + Muc_v*Bvuc + Bvuc'*Mv_uc + Bvuc'*Mvv*Bvuc;
		//	Mr_uc = Muc_uc + Muc_v*Bvuc + (Muc_v*Bvuc)' + (Bvuc'*Mvv)*Bvuc;

		//	Fr_uc = Fuc + Muc_v*bprim + Bvuc'*Fv + Bvuc'*Mvv*bprim;
		//	Fr_uc = Fuc + (Muc_v+ Bvuc'*Mvv)*bprim + Bvuc'*Fv ;

		// BtMvu = Bvuc'*Mv_uc
		nL = nC = lds->iquc[0];
		nk = s->nqv;
		for(i=1;i<=nL;i++)
		{
			for(j=1;j<=nC;j++)
			{
				term = 0.0;
				for(k=1;k<=nk;k++)
				{
					term += lds->Bvuc[k][i] * lds->M[s->qv[k]][lds->iquc[j]];
				}
				lds->BtMvu[i][j] = term;
			}
		}

		// BtMvv = Bvuc'*Mvv
		nL = lds->iquc[0];
		nC = nk = s->nqv;
		for(i=1;i<=nL;i++)
		{
			for(j=1;j<=nC;j++)
			{
				term = 0.0;
				for(k=1;k<=nk;k++)
				{
					term += lds->Bvuc[k][i] * lds->M[s->qv[k]][s->qv[j]];
				}
				lds->BtMvv[i][j] = term;
			}
		}

		// BtFv = Bvuc'*Fv
		nL = lds->iquc[0];
		nk = s->nqv;
		for(i=1;i<=nL;i++)
		{
			term = 0.0;
			for(k=1;k<=nk;k++)
			{
				term += lds->Bvuc[k][i] * lds->F[s->qv[k]];
			}
			lds->BtFv[i] = term;
		}

		// BtMB = BtMvv*Bvuc
		nL = nC = lds->iquc[0];
		nk = s->nqv;
		for(i=1;i<=nL;i++)
		{
			for(j=1;j<=nC;j++)
			{
				term = 0.0;
				for(k=1;k<=nk;k++)
				{
					term += lds->BtMvv[i][k] * lds->Bvuc[k][j];
				}
				lds->BtMB[i][j] = term;
			}
		}

		// MBMb = (Muv+Bvuc'*Mvv)*bprim
		nL = lds->iquc[0];
		nk = s->nqv;
		for(i=1;i<=nL;i++)
		{
			term = 0.0;
			for(k=1;k<=nk;k++)
			{
				term += (lds->M[lds->iquc[i]][s->qv[k]] + lds->BtMvv[i][k]) * lds->bp[k];
			}
			lds->MBMb[i] = term;
		}

/*
		nL = lds->iquc[0];
		nk = s->nqv;
		for(i=1;i<=nL;i++)
		{
			// BtMvu = Bvuc'*Mvu
			nC = lds->iquc[0];
			for(j=1;j<=nC;j++)
			{
				term = 0.0;
				for(k=1;k<=nk;k++)
				{
					term += lds->Bvuc[k][i] * lds->M[s->qv[k]][lds->iquc[j]];
				}
				lds->BtMvu[i][j] = term;
			}

			// BtMvv = Bvuc'*Mvv
			nC = s->nqv;
			for(j=1;j<=nC;j++)
			{
				term = 0.0;
				for(k=1;k<=nk;k++)
				{
					term += lds->Bvuc[k][i] * lds->M[s->qv[k]][s->qv[j]];
				}
				lds->BtMvv[i][j] = term;
			}

			// BtFv = Bvuc'*Fv
			term = 0.0;
			for(k=1;k<=nk;k++)
			{
				term += lds->Bvuc[k][i] * lds->F[s->qv[k]];
			}
			lds->BtFv[i] = term;

			// BtMB = BtMvv*Bvuc
			nC = lds->iquc[0];
			for(j=1;j<=nC;j++)
			{
				term = 0.0;
				for(k=1;k<=nk;k++)
				{
					term += lds->BtMvv[i][k] * lds->Bvuc[k][j];
				}
				lds->BtMB[i][j] = term;
			}

			// MBMb = (Muv+Bvuc'*Mvv)*bprim
			nC = s->nqv;
			term = 0.0;
			for(k=1;k<=nk;k++)
			{
				term += (lds->M[lds->iquc[i]][s->qv[k]] + lds->BtMvv[i][k]) * lds->bp[k];
			}
			lds->MBMb[i] = term;
		}
*/

		// Mruc Fruc
		nL = nC = lds->iquc[0];
		for(i=1;i<=nL;i++)
		{
			for(j=1;j<=nC;j++)
			{
				lds->Mruc[i][j] = lds->M[lds->iquc[i]][lds->iquc[j]] +
					lds->BtMvu[i][j] + lds->BtMvu[j][i] +
					lds->BtMB[i][j];
			}

			lds->Fruc[i] = lds->F[lds->iquc[i]] + lds->MBMb[i] + lds->BtFv[i];
		}
	}
	else
	{
		// Mruc Fruc
		nL = nC = lds->iquc[0];
		for(i=1;i<=nL;i++)
		{
			for(j=1;j<=nC;j++)
			{
				lds->Mruc[i][j] = lds->M[lds->iquc[i]][lds->iquc[j]];
			}
			lds->Fruc[i] = lds->F[lds->iquc[i]];
		}
	}

	// Mr Fr
	nL = nC = s->nqu;

	for(i=1;i<=nL;i++)
	{
		has_a_line_of_zeros = 1;
		for(j=1;j<=nC;j++)
		{
			lds->Mr[i][j] = lds->Mruc[i][j]; // Muu
			if (lds->Mr[i][j]>1e-16)
				has_a_line_of_zeros = 0;
		}
		if (has_a_line_of_zeros)
		{
			printf("The line %d of the reduced mass matrix, associated to q(%d), is full of zeros\n",i,lds->iquc[i]);
			for(k=1;k<=nC;k++)
				printf("lds->Mr[%d][%d] = %e;\n",i,k,lds->Mr[i][k]);
			fprintf(stderr,"The reduced mass matrix has a line of zeros\n");
		}

		term = 0.0;
		for(k=1;k<=s->nqc;k++)
		{
			term += lds->Mruc[i][s->nqu+k] * s->qdd[lds->iquc[s->nqu+k]];
		}
		lds->Fr[i] = -(lds->Fruc[i] + term);
	}


	// calcul des accelerations reduites : 'resolution' du systeme ODE = Mr*qddu = Fr;
	choldc(lds->Mr,s->nqu,lds->p_Mr);
	cholsl(lds->Mr,s->nqu,lds->p_Mr,lds->Fr,s->qddu);

	nL = s->nqu;
	for(i=1;i<=nL;i++)
	{
		s->qdd[s->qu[i]] = s->qddu[i];
	}

	if (s->nqv>0)
	{
		// qdd_v = Bvuc*qdd_u + bp
		nL = s->nqv;
		nk = s->nqu;
		for(i=1;i<=nL;i++)
		{
			term = 0.0;
			for(k=1;k<=nk;k++)
			{
				term += lds->Bvuc[i][k] * s->qddu[k];
			}
			s->qdd[s->qv[i]] = term + lds->bp[i];
		}
	}

	// Qc = Mruc_cu * qddu + Mruc_cc * qddc + Fruc_c
	if (s->nqc>0)
	{
		nL = s->nqc;
		for(i=1;i<=nL;i++)
		{
			term = 0.0;

			nk = s->nqu;
			for(k=1;k<=nk;k++)
			{
				term += lds->Mruc[s->nqu+i][k] * s->qddu[k];
			}

			nk = s->nqc;
			for(k=1;k<=nk;k++)
			{
				term += lds->Mruc[s->nqu+i][s->nqu+k] * s->qdd[lds->iquc[s->nqu+k]];
			}

			lds->Qc[i] = term + lds->Fruc[s->nqu+i];
		}
	}


	return iter;
}
#endif
