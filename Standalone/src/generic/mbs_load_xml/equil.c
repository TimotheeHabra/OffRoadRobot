#include "equil.h"

int compute_Fr_uc( double *Fr_uc, MDS_gen_strct *mds_gen_strct, PART_gen_strct *part_gen_strct,	LocalDataStruct *lds, MBSdataStruct *MBSdata )
{
	int ok, fail = 0; 
	int i, j; 

	int *ind_uc; 
	int n_uc; 

	double *c, *Qq; // change it
	double **Bvuc;

	double *F;// *Fr_uc;
	double val;
	int ind_x;

	n_uc = part_gen_strct->n_qu + mds_gen_strct->bodytree->n_qc;
	ind_uc = get_int_vec(n_uc); 
	conc_int_vec(part_gen_strct->ind_qu, part_gen_strct->n_qu, mds_gen_strct->bodytree->qc, mds_gen_strct->bodytree->n_qc, ind_uc);

	F = get_double_vec(mds_gen_strct->bodytree->n_joint);


	c = get_double_vec(mds_gen_strct->bodytree->n_joint); // change it
	Qq = get_double_vec(mds_gen_strct->bodytree->n_joint);
	Bvuc = get_double_tab(part_gen_strct->n_qv, n_uc);

	// Copie des variables d'équilibre "sensibles" dans les variables articulaires indépendantes
	/*
	for(i=0; i<n_x; i++)
	{
		equil_gen_strct->xeq_ptr[i][0] = x[i];
	}*/

	//   II. Process

	//   1. Expression des variables commandées

	if(mds_gen_strct->bodytree->n_qc > 0)
	{
		user_DrivenJoints(MBSdata, MBSdata->tsim);          // appel routine Robotran           
		for(i=0; i<mds_gen_strct->bodytree->n_qc; i++)  // warning a envisager si diff de zero
		{
			if (MBSdata->qdd[mds_gen_strct->bodytree->qc[i]+1] != 0.0) // +1 for convention change it
			{
				MBSdata->qdd[mds_gen_strct->bodytree->qc[i]+1] = 0.0; // +1 for convention
				printf(">>MBS>> MBS_equil - user file : non-zero driven accelerations : forced to zero");
			}
		}
	}

	//   2. Contraintes : résolution géométrique

	if(part_gen_strct->n_qv > 0)
	{
		ok = mbs_close_geo(MBSdata, lds);
		if (ok == -1)
		{
			fail = 1;
			
			free_int_vec(ind_uc ); 
			free_double_vec(F);
			free_double_vec(Fr_uc);

			free_double_vec(c); // change it
			free_double_vec(Qq);
			free_double_tab(Bvuc, part_gen_strct->n_qv);

			return fail;
		}
	}

	//   3. Containtes : résolution cinématique

	if(part_gen_strct->n_qv > 0)
	{
		mbs_close_kin(MBSdata, lds);
	}

	//   4. Forces sur le système : extérieures,de lien,articulaires

	//mbs_calc_force; ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for(i=1;i<=MBSdata->nbody;i++)
	{
		for(j=1;j<=3;j++)
		{
			MBSdata->frc[j][i]=0.0;
			MBSdata->trq[j][i]=0.0;
		}
	}

	if(MBSdata->Nlink > 0) link(MBSdata->frc,MBSdata->trq,MBSdata->Fl,MBSdata->Z,MBSdata->Zd,MBSdata,MBSdata->tsim);

	if(MBSdata->Nxfrc > 0) extforces(MBSdata->frc,MBSdata->trq,MBSdata,MBSdata->tsim);

	MBSdata->Qq = user_JointForces(MBSdata,MBSdata->tsim);


	//   5.  Equations of motion (unconstrained MBS)

	zeros_double_vec(&(MBSdata->qdd[1]), MBSdata->njoint); // ... sécurité en attente d'une fonction symb c(q,qd)     
	
	dirdyna(lds->M,lds->c,MBSdata,MBSdata->tsim); // futur : c(q,qd)
     
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	copy_double_vec(&(lds->c[1]), c, mds_gen_strct->bodytree->n_joint); // change it
	copy_double_vec(&(MBSdata->Qq[1]), Qq, mds_gen_strct->bodytree->n_joint);

	for(i=0; i<part_gen_strct->n_qv; i++)
	{
		for(j=0; j<n_uc; j++)
		{
			Bvuc[i][j] = lds->Bvuc[i+1][j+1];
		}
	}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	for(i=0; i<mds_gen_strct->bodytree->n_joint; i++)
	{
		F[i] = c[i] - Qq[i];
	}

	//   6.  Equations of motion (reduction (for constrained MBS)

	if (part_gen_strct->n_qv > 0) // Fr_uc = F(ind_uc) + Bvuc'*F(ind_v);
	{
		for(i=0; i<n_uc; i++)
		{
			val = 0.0;
			for(j=0; j<part_gen_strct->n_qv; j++)
			{
				val += Bvuc[j][i] * F[part_gen_strct->ind_qv[j]];
			}
			Fr_uc[i] = F[ind_uc[i]] + val;
		}
	}
	else
	{
		for(i=0; i<n_uc; i++)
		{
			Fr_uc[i] = F[ind_uc[i]];
		}
	}

	//   7:  Calcul du résidu : Fu (= argument de sortie)
	/*
	ind_x = 0;
	for(i=0; i<part_gen_strct->n_qu; i++)
	{
		if (find_int_vec(equil_gen_strct->options->ignored_qu,equil_gen_strct->options->n_ignored_qu, part_gen_strct->ind_qu[i]) == -1)
		{
			Fu[ind_x] = Fr_uc[i];
			ind_x++;
		}
	}

	// adding the extra function 
	(*equil_gen_strct->options->EQUIL_extra_fct_ptr)(MBSdata,&(Fu[ind_x]));
	*/
	
/*
	//   8:  Calcul des multiplicateurs de Lagrange

	if(part_gen_strct->n_v > 0)
	{
		MBSdata->lambda = zeros(length(ind_cont),1);
		Jv = J(ind_cont, ind_v);    Fv = F(ind_v);
		lam = (Jv')\Fv;
		MBSdata->lambda(ind_cont) = lam;
	}
	else
	{
		MBSdata->lambda = [];
	}

	//   9:  Forces/couples associées aux commandes : Q(ind_c)

	if(part_gen_strct->n_v> 0)
	{
		nuc = length(ind_uc);
		Fc = Fr_uc(nqu+1:nuc);
		MBSdata->Qq(ind_c) = Fc;
	}

	/*
	if(STORE) // faire une option store en plus ob1
	{
		MBS_equil.history.niter = MBS_equil.history.niter + 1;
		MBS_equil.history.q = [MBS_equil.history.q;MBSdata->q'];
		MBS_equil.history.xeq = [MBS_equil.history.xeq;x'];
		MBS_equil.history.Fr = [MBS_equil.history.Fr;norm(Fu)];
		// output user
		iter = MBS_equil.history.niter;
		callfct.user.out(MBS_data,iter);
	}

	if( strcmp(MBS_equil.opt.visualize,'yes'))
	{
		mbs_visualize('update');
	}
	
	*/

	free_int_vec(ind_uc ); 
	free_double_vec(F);

	free_double_vec(c); // change it
	free_double_vec(Qq);
	free_double_tab(Bvuc, part_gen_strct->n_qv);

	return fail; 

}


int EQUIL_fct_equil(double *x, int n_x, double *Fu, MDS_gen_strct *mds_gen_strct, PART_gen_strct *part_gen_strct, EQUIL_gen_strct *equil_gen_strct,	LocalDataStruct *lds, MBSdataStruct *MBSdata )
{
	int fail = 0; 
	int i, j; 

	int ind_x;
	double *Fr_uc;
	Fr_uc = get_double_vec(part_gen_strct->n_qu + mds_gen_strct->bodytree->n_qc);


	for(i=0; i<n_x; i++)
	{
		equil_gen_strct->xeq_ptr[i][0] = x[i];
	}

	fail = compute_Fr_uc( Fr_uc, mds_gen_strct,part_gen_strct, lds, MBSdata );

	// compute mbs rest ("residu" in french)

	ind_x = 0;
	for(i=0; i<part_gen_strct->n_qu; i++)
	{
		if (find_int_vec(equil_gen_strct->options->ignored_qu,equil_gen_strct->options->n_ignored_qu, part_gen_strct->ind_qu[i]) == -1)
		{
			Fu[ind_x] = Fr_uc[i];
			ind_x++;
		}
	}

	// adding the extra function 
	(*equil_gen_strct->options->EQUIL_extra_fct_ptr)(MBSdata,&(Fu[ind_x]));


	free_double_vec(Fr_uc);
	return fail; 
}

void choldc_double_tab(double **a, int n, double *p)
{
	//void nrerror(char error_text[]);
	int i,j,k;
	double sum;

	for (i=0;i<n;i++)
	{
		for (j=i;j<n;j++)
		{
			for (sum=a[i][j],k=i-1;k>=0;k--) 
			{
				sum -= a[i][k]*a[j][k];
			}
			if (i == j) 
			{
				if (sum <= 0.0)
				{
					//nrerror("choldc failed");
					printf("choldc failed/n");
				}
				p[i]=sqrt(sum);
			} 
			else 
			{
				a[j][i]=sum/p[i];
			}
		}
	}
}
void cholsl_double_tab(double **a, int n, double *p, double *b, double *x)
{
	int i,k;
	double sum;

	for (i=0;i<n;i++) 
	{
		for (sum=b[i],k=i-1;k>=0;k--) 
		{
			sum -= a[i][k]*x[k];
		}
		x[i]=sum/p[i];
	}
	for (i=n-1;i>=0;i--) 
	{
		for (sum=x[i],k=i+1;k<n;k++) 
		{
			sum -= a[k][i]*x[k];
		}
		x[i]=sum/p[i];
	}
}

#define NRANSI
#define TINY 1.0e-20

void ludcmp_2(double **a, int n, int *indx, double *d)
{
	int i,imax,j,k;
	double big,dum,sum,temp;
	double *vv;

	imax = 0;

	vv=get_double_vec(n); 
	*d=1.0;
	for (i=0;i<n;i++) {
		big=0.0;
		for (j=0;j<n;j++)
			if ((temp=fabs(a[i][j])) > big) big=temp;
		if (big == 0.0) 
		{
			//nrerror("Singular matrix in routine ludcmp");
			printf("Singular matrix in routine ludcmp\n");
		}
		vv[i]=1.0/big;
	}
	for (j=0;j<n;j++) {
		for (i=0;i<j;i++) {
			sum=a[i][j];
			for (k=0;k<i;k++) sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0;
		for (i=j;i<n;i++) {
			sum=a[i][j];
			for (k=0;k<j;k++)
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
			if ( (dum=vv[i]*fabs(sum)) >= big) {
				big=dum;
				imax=i;
			}
		}
		if (j != imax) {
			for (k=0;k<n;k++) {
				dum=a[imax][k];
				a[imax][k]=a[j][k];
				a[j][k]=dum;
			}
			*d = -(*d);
			vv[imax]=vv[j];
		}
		indx[j]=imax;
		if (a[j][j] == 0.0) a[j][j]=TINY;
		if (j != n-1) {
			dum=1.0/(a[j][j]);
			for (i=j+1;i<n;i++) a[i][j] *= dum;
		}
	}
	free_double_vec(vv);
}
#undef TINY
#undef NRANSI

void lubksb_2(double **a, int n, int *indx, double b[])
{
	int i,ii=-1,ip,j;
	double sum;

	for (i=0;i<n;i++) {
		ip=indx[i];
		sum=b[ip];
		b[ip]=b[i];
		if (ii != -1)
			for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j];
		else if (sum) ii=i;
		b[i]=sum;
	}
	for (i=n-1;i>=0;i--) {
		sum=b[i];
		for (j=i+1;j<n;j++) sum -= a[i][j]*b[j];
		b[i]=sum/a[i][i];
	}
}


int EQUIL_fsolvepk(int (*fun_ptr)(double*, int, double*, MDS_gen_strct*, PART_gen_strct*, EQUIL_gen_strct*, LocalDataStruct*, MBSdataStruct*), MDS_gen_strct *mds_gen_strct, PART_gen_strct *part_gen_strct, EQUIL_gen_strct *equil_gen_strct,	LocalDataStruct *lds, MBSdataStruct *MBSdata )
{ //
	
	int i, j, k; 
	int equil_fail = 0;
	int sens_ok;

	int grad_fun_ptr = NULL; // dead function at this time

	int n_x, n_f;
	double  *xd, *f, *fd;
	double **grad;

	double norm_pk, normR; 
	int it_cou, r_cou;

	int ind; 

	double **grad_x_sens;
	double *xd_x_sens, *x_x_sens, *p_grad_x_sens, *f_x_sens, *xdR, *xR;
	double norm1, norm2, inorm2, dnorm2, norm3, sc_prdct;

	int *indx;
	double d;
	
	n_x = equil_gen_strct->n_x; 
	n_f = n_x; // in this case only

	xd = get_double_vec(n_x);
	f = get_double_vec(n_x);
	fd = get_double_vec(n_x);
	grad = get_double_tab(n_f,n_x);
	
	// Input arguments

	/*
	if nargin<4
		grad_fun_ptr=[];
		if nargin<3,			OPTIONS=[];				end
	end
	if isempty(OPTIONS), 	OPTIONS=zeros(21,1); end
	if length( OPTIONS)<21,  OPTIONS(21)=0   ;    end
	if OPTIONS(2 )==0, 		OPTIONS(2) =1e-6; 	end		// std. precision for solution
	if OPTIONS(3 )==0, 		OPTIONS(3) =1e-6; 	end		// std. precision for solution
	if OPTIONS(4 )==0, 		OPTIONS(4) =0.8 ; 	end		// std. relaxation
	if OPTIONS(5 )==0, 		OPTIONS(5) =1e-6; 	end		// std. deviation for jacobian
	if OPTIONS(14)==0, 		OPTIONS(14)= 50 ; 	end		// std. number of iteration
	//*/

	// Check if gradient function name is set correctly

	if(grad_fun_ptr != NULL)
	{
		/*if ~isstr(grad_fun_ptr)
			printf(">>MBS>> The user-supplied gradient (Jacobian) must be a string.");
			equil_fail = 1;
			return;
		end
		ff = exist(grad_fun_ptr,'file');
		if ff ~= 2
			printf(">>MBS>> fsolvepk: The user-supplied gradient function doesn''t exist");
			equil_fail = 1;
			return;       
		end
		mygrad = str2func(grad_fun_ptr);*/
	}

	// Gradient test - search of sensitive variables

	equil_fail = (*fun_ptr)(equil_gen_strct->x, n_x, f, mds_gen_strct, part_gen_strct, equil_gen_strct, lds, MBSdata ); // caution no store no store opt 


	if(equil_fail)
	{
		return equil_fail;
	}
	if (grad_fun_ptr != NULL)        // analytical evaluation
	{
		//[grad] = mygrad(x);
	}
	else                        // numerical estimation
	{
		for(i=0; i<n_x; i++)
		{
			copy_double_vec(equil_gen_strct->x, xd, n_x);
			if(fabs(equil_gen_strct->x[i])>1e-2)
			{
				xd[i] = equil_gen_strct->x[i] * (1.0 + equil_gen_strct->options->devjac);
			}
			else
			{
				xd[i] = equil_gen_strct->x[i] + equil_gen_strct->options->devjac;
			}
			equil_fail = (*fun_ptr)(xd, n_x, fd, mds_gen_strct, part_gen_strct, equil_gen_strct, lds, MBSdata ); // caution no store no store opt 
			if (equil_fail)
			{
				return equil_fail;
			}
			for(j=0; j<n_f; j++)
			{
				grad[j][i] = (fd[j]-f[j]) / (xd[i]-equil_gen_strct->x[i]);
			}
		}
	}

	for(i=0; i<n_x; i++)									// testing the gradient
	{
		sens_ok = 0;
		for(j=0; j<n_f; j++)
		{
			if(fabs(grad[j][i]) > fabs(equil_gen_strct->options->senstol))
			{
				sens_ok = 1;
			}
		}
		if(sens_ok)	
		{
			equil_gen_strct->x_sens[equil_gen_strct->n_x_sens] = i;
			(equil_gen_strct->n_x_sens)++;
		}
		else   // non sensible outputs
		{
			equil_gen_strct->x_non_sens[equil_gen_strct->n_x_non_sens] = i;
			(equil_gen_strct->n_x_non_sens)++;
			for(j=0; j<n_f; j++)
			{
				grad[j][i] = 0.0;
			}
		}			
	}
	if(rank_double_tab(grad, n_f, n_x) != equil_gen_strct->n_x_sens)  // gradient singularity
	{
		printf(">>MBS>> fsolvepk: After elimintating the non sensitive variables, the numerical gradient is still singular !\n");
		equil_fail = 1;
		return equil_fail;
	}		
	if(equil_gen_strct->n_x_sens == 0)
	{
		printf("fsolvepk: none of the proposed parameter are sensitive !\n");
		equil_fail = 0;
		return equil_fail;
	}	

	grad_x_sens = get_double_tab(equil_gen_strct->n_x_sens, equil_gen_strct->n_x_sens);
	xd_x_sens = get_double_vec(equil_gen_strct->n_x_sens);
	x_x_sens = get_double_vec(equil_gen_strct->n_x_sens);
	p_grad_x_sens = get_double_vec(equil_gen_strct->n_x_sens);
	f_x_sens = get_double_vec(equil_gen_strct->n_x_sens);
	xdR = get_double_vec(equil_gen_strct->n_x_sens);
	xR = get_double_vec(equil_gen_strct->n_x_sens);

	// Solution searching
                    
	equil_fail = (*fun_ptr)(equil_gen_strct->x, n_x, f, mds_gen_strct, part_gen_strct, equil_gen_strct, lds, MBSdata ); // caution store // toute la fonction
	if(equil_fail)
	{
		return equil_fail; 
	}

	norm_pk = norm_double_vec(f, n_f); // value of nonacceptance,

	it_cou=0;// iteration counter initialisation
	r_cou = 0;
	normR = DBL_MAX; 		

	while( (norm_pk > equil_gen_strct->options->equitol) && (it_cou<equil_gen_strct->options->itermax) )
	{
		if (equil_gen_strct->options->verbose)
		{
			printf(">>MBS>> ---> Residue = %f\n",norm_pk);
		}
		// gradient value estimation,
		zeros_double_tab(grad, n_f, n_x);
		if (grad_fun_ptr != NULL)                // analytical evaluation
		{
			//[grad] = mygrad(x);
		}
		else								// numerical estimation
		{
			for(i=0; i<equil_gen_strct->n_x_sens; i++)
			{
				ind = equil_gen_strct->x_sens[i];
				for(j=0; j<equil_gen_strct->n_x_sens; j++)
				{
					xd[equil_gen_strct->x_sens[j]] = equil_gen_strct->x[equil_gen_strct->x_sens[j]];
				}
				for(j=0; j<equil_gen_strct->n_x_non_sens; j++)
				{
					xd[equil_gen_strct->x_non_sens[j]] = 0.0;  // mise a zero des variables insensibles
				}     
				if(fabs(equil_gen_strct->x[ind])>1e-2)
				{
					xd[ind] = equil_gen_strct->x[ind] * (1+equil_gen_strct->options->devjac);
				}
				else
				{
					xd[ind] = equil_gen_strct->x[ind] + equil_gen_strct->options->devjac;
				}
				equil_fail = (*fun_ptr)(xd, n_x, fd, mds_gen_strct, part_gen_strct, equil_gen_strct, lds, MBSdata ); // caution no store opt
				if(equil_fail)
				{
					return equil_fail;
				}
				for(j=0; j<n_f; j++)
				{
					grad[j][ind]=(fd[j]-f[j])/(xd[ind]-equil_gen_strct->x[ind]);
				}
			}
		}


		for(i=0; i<equil_gen_strct->n_x_sens; i++)
		{
			for(j=0; j<equil_gen_strct->n_x_sens; j++)
			{
				grad_x_sens[i][j] = grad[equil_gen_strct->x_sens[i]][equil_gen_strct->x_sens[j]];
			}
		}

		slct_double_vec(f,n_f,equil_gen_strct->x_sens,equil_gen_strct->n_x_sens,f_x_sens);


		//xd(x_sens)=grad(x_sens,x_sens)\f(x_sens);
		copy_double_vec(f_x_sens, xd_x_sens, equil_gen_strct->n_x_sens);
		indx = get_int_vec(equil_gen_strct->n_x_sens);
		ludcmp_2(grad_x_sens, equil_gen_strct->n_x_sens, indx , &d);
		lubksb_2(grad_x_sens, equil_gen_strct->n_x_sens, indx , xd_x_sens);


		//save_double_vec(xd_x_sens, equil_gen_strct->n_x_sens,"xd_sens.txt");

		for(i=0; i<equil_gen_strct->n_x_sens; i++)
		{
			xd[equil_gen_strct->x_sens[i]] = xd_x_sens[i];
		}
			
/*
	printf("=======================================================================================================\n");	
	printf(" 1er call : x\n"); 
	print_double_vec(equil_gen_strct->x, n_x);
	printf(" 1er call : f\n"); 
	print_double_vec(f, n_f);
	printf(" 1er call : xd\n"); 
	print_double_vec(xd, n_x);
	printf(" 1er call : fd\n"); 
	print_double_vec(fd, n_f);
	printf(" 1er call : grad\n");
	save_double_tab(grad, n_f, n_x, "grad3.txt");*/

		// softing
    
		if(equil_gen_strct->options->smooth)
		{
			slct_double_vec(equil_gen_strct->x, n_x,equil_gen_strct->x_sens,equil_gen_strct->n_x_sens, x_x_sens);
			norm1=norm_double_vec(x_x_sens, equil_gen_strct->n_x_sens);
			norm2=norm_double_vec(xd_x_sens, equil_gen_strct->n_x_sens);
			inorm2=norm2;
			dnorm2=norm2;
			if(inorm2< 1e-6)
			{
				norm2 = 1.0;
				dnorm2 = 1.0;
			}
			if((norm1< 1e-2) && (inorm2>1e-1))
			{
				dnorm2 = 1e-1;
			}
			if((norm1>=1e-2) && ((inorm2/norm1)>0.8))
			{
				sc_prdct = 0.0;
				for(i=0; i<n_x; i++)
				{
					sc_prdct += xd[i] * equil_gen_strct->x[i];
				}
				norm3= fabs(sc_prdct/(norm1*norm2));	// error perpendicular to actual position
				if(norm3 < 0.3)
				{
					dnorm2 = norm1 * 1e-1;
				}
				else
				{
					dnorm2 = norm1 * 0.8;
				}
			}
		
			if ((norm1>=1e-2) && (inorm2/norm1>300))
			{
				dnorm2= 1e-1 * norm1; 		// "grad" singularity possible
			}
			for(i=0; i< equil_gen_strct->n_x_sens; i++)
			{
				xd[equil_gen_strct->x_sens[i]] = (dnorm2/norm2) * xd[equil_gen_strct->x_sens[i]];
			} 
			slct_double_vec(xd, n_x,equil_gen_strct->x_sens,equil_gen_strct->n_x_sens, xdR);
		}
		else
		{
			slct_double_vec(xd, n_x,equil_gen_strct->x_sens,equil_gen_strct->n_x_sens, xdR);
			norm2=norm_double_vec(xdR, equil_gen_strct->n_x_sens);
			dnorm2=norm2;
		}
    
		// NR iteration
		
		for(i=0; i< equil_gen_strct->n_x_sens; i++)
		{
			equil_gen_strct->x[equil_gen_strct->x_sens[i]] = equil_gen_strct->x[equil_gen_strct->x_sens[i]] - xd[equil_gen_strct->x_sens[i]];											// new values of parameters
		}
		equil_fail = (*fun_ptr)(equil_gen_strct->x, n_x, f, mds_gen_strct, part_gen_strct, equil_gen_strct, lds, MBSdata ); // caution  store =1
		if(equil_fail)
		{
			return equil_fail;
		}
		norm_pk = norm_double_vec(f,n_f);								// value of nonacceptance,
  
		// relaxation
    
		while (norm_pk > normR && r_cou < 10)
		{
			for(i=0; i< equil_gen_strct->n_x_sens; i++)
			{
			xdR[i] = equil_gen_strct->options->relax * xdR[i];
			equil_gen_strct->x[equil_gen_strct->x_sens[i]]=xR[i]-xdR[i];
			}
			r_cou = r_cou + 1; 	  		// relaxation
			equil_fail = (*fun_ptr)(equil_gen_strct->x, n_x, f, mds_gen_strct, part_gen_strct, equil_gen_strct, lds, MBSdata ); // caution  store =0
			if(equil_fail)
			{
				return equil_fail;
			}
			norm_pk =norm_double_vec(f,n_f);															// value of nonacceptance,
			if(equil_gen_strct->options->verbose)
			{
				printf(">>MBS>> ----> ... Relaxation - Residue = %f\n", norm_pk);
			}
		}
		it_cou = it_cou + 1;
		if (dnorm2==norm2)
		{
			normR = 0.9*norm_pk;
		}
		else
		{
			normR=norm_pk;
		}
		slct_double_vec(equil_gen_strct->x, n_x,equil_gen_strct->x_sens,equil_gen_strct->n_x_sens, xR);
		r_cou = 0;		 	// iteration counter actualisation
	}
	if(equil_gen_strct->options->verbose)
	{
		printf(">>MBS>> ---> Residue = %f\n", norm_pk);
	}
	if(it_cou >= equil_gen_strct->options->itermax)
	{
		printf(">>MBS>> Equilibrium has not converged after %d iterations\n", equil_gen_strct->options->itermax);
		equil_fail = 1;
		return equil_fail;
	}

	return equil_fail; 
}



int EQUIL_run_equil(MDS_gen_strct *mds_gen_strct, PART_gen_strct *part_gen_strct, EQUIL_gen_strct *equil_gen_strct,	LocalDataStruct *lds, MBSdataStruct *MBSdata)
{
    int equil_fail = 1, test, i ; 
	int ind_x, ind_c;
	double *qd_save; 

	int (* EQUIL_fct_equil_ptr)(double *, int, double*, MDS_gen_strct*, PART_gen_strct*, EQUIL_gen_strct*, LocalDataStruct*, MBSdataStruct*); 
	EQUIL_fct_equil_ptr = EQUIL_fct_equil;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Test de prerequis

	if(MBSdata->DonePart == 0 && MBSdata->Ncons > 0)
	{
		printf(">>MBS>> mbs_run_equil : please perform a partitioning at first !\n");
		equil_fail = 1;
		return !equil_fail;
	}
	if(part_gen_strct->n_qu == 0)
	{
		printf(">>MBS>> mbs_run_equil : no independent variable - irrelevant process !\n");
		equil_fail = 1;
		return !equil_fail;
	}
	if(equil_gen_strct->options->static_equil)
	{
		qd_save = get_double_vec(mds_gen_strct->bodytree->n_joint);
		copy_double_vec(&(MBSdata->qd[1]), qd_save, mds_gen_strct->bodytree->n_joint);
		test = 0;
		for(i=0; i<part_gen_strct->n_qu; i++)
		{
			if(MBSdata->qd[part_gen_strct->ind_qu[i]+1] != 0.0)
			{
				test = 1;
				MBSdata->qd[part_gen_strct->ind_qu[i]+1] = 0.0;
			}
		}
		for(i=0; i<part_gen_strct->n_qv; i++)
		{
			if(MBSdata->qd[part_gen_strct->ind_qv[i]+1] != 0.0)
			{
				test = 1;
				MBSdata->qd[part_gen_strct->ind_qv[i]+1] = 0.0;
			}
		}
		if(test)
		{
			printf(">>MBS>> mbs_run_equil : non-zero initial non-driven velocities have been forced to 0 !\n");
		}
	}
	test = 0;
	for(i=0; i<part_gen_strct->n_qu; i++)
	{
		if(MBSdata->qdd[part_gen_strct->ind_qu[i]+1] != 0.0)
		{
			test = 1;
			MBSdata->qdd[part_gen_strct->ind_qu[i]+1] = 0.0;
		}
	}
	for(i=0; i<part_gen_strct->n_qv; i++)
	{
		if(MBSdata->qdd[part_gen_strct->ind_qv[i]+1] != 0.0)
		{
			test = 1;
			MBSdata->qdd[part_gen_strct->ind_qv[i]+1] = 0.0;
		}
	}
	if(test)
	{
		printf(">>MBS>> mbs_run_equil : non-zero initial non-driven accelerations have been forced to 0 !\n");
	}
	if(equil_gen_strct->options->verbose)
	{
		printf(">>MBS>> ***** mbs_run_equil begin *****\n");
	}

	// Noms des parametres et Valeurs initiales x0 (rangé en une colonne)

	equil_gen_strct->n_x = part_gen_strct->n_qu - equil_gen_strct->options->n_ignored_qu + equil_gen_strct->options->n_x_extra;

	/*
	if (equil_gen_strct->options->static_equil)
	{
		equil_gen_strct->n_x =  (part_gen_strct->n_qu - equil_gen_strct->options->n_ignored_qu);
	}
	else 
	{
		equil_gen_strct->n_x =  2 * (part_gen_strct->n_qu - equil_gen_strct->options->n_ignored_qu);
	}
	*/
	equil_gen_strct->x = get_double_vec(equil_gen_strct->n_x);
	equil_gen_strct->x_sens = get_int_vec(equil_gen_strct->n_x); // over sized
	equil_gen_strct->x_non_sens = get_int_vec(equil_gen_strct->n_x); // over sized

	equil_gen_strct->xeq_ptr = (double**) malloc(equil_gen_strct->n_x*sizeof(double*));
	

	ind_x = 0;
	ind_c = 0;
	for(i=0; i< part_gen_strct->n_qu; i++)
	{		
		if (find_int_vec(equil_gen_strct->options->ignored_qu,equil_gen_strct->options->n_ignored_qu, part_gen_strct->ind_qu[i]) == -1)
		{
			ind_c = find_int_vec(equil_gen_strct->options->changed_qu,equil_gen_strct->options->n_changed_qu, part_gen_strct->ind_qu[i]);
			if (ind_c == -1)
			{
				equil_gen_strct->xeq_ptr[ind_x] = & (MBSdata->q[part_gen_strct->ind_qu[i]+1]); // change it ; +1 is for old convention
				equil_gen_strct->x[ind_x] = (MBSdata->q[part_gen_strct->ind_qu[i]+1]); // change it ; +1 is for old convention
				/*if (!equil_gen_strct->options->static_equil)
				{
					equil_gen_strct->xeq_ptr[equil_gen_strct->n_x /2 + ind_x] = & (MBSdata->qd[part_gen_strct->ind_qu[i]+1]); // change it ; +1 is for old convention
					equil_gen_strct->x[equil_gen_strct->n_x /2 + ind_x] = (MBSdata->qd[part_gen_strct->ind_qu[i]+1]); // change it ; +1 is for old convention
				}//*/
			}
			else
			{
				equil_gen_strct->xeq_ptr[ind_x] = equil_gen_strct->options->x_subst_ptr[ind_c];
				equil_gen_strct->x[ind_x] = *equil_gen_strct->options->x_subst_ptr[ind_c];
				/*if (!equil_gen_strct->options->static_equil)
				{
					equil_gen_strct->xeq_ptr[equil_gen_strct->n_x /2 + ind_x] = & (MBSdata->qd[part_gen_strct->ind_qu[i]+1]); // change it ; +1 is for old convention
					equil_gen_strct->x[equil_gen_strct->n_x /2 + ind_x] = (MBSdata->qd[part_gen_strct->ind_qu[i]+1]); // change it ; +1 is for old convention
				}//*/
			}
			ind_x++;
		}
	}
	for(i=0; i<equil_gen_strct->options->n_x_extra; i++)
	{
		equil_gen_strct->xeq_ptr[ind_x] = equil_gen_strct->options->x_extra_ptr[i];
		equil_gen_strct->x[ind_x] = *equil_gen_strct->options->x_extra_ptr[i];
		ind_x++;
	}


	// Appel routine user_io pour initialisation
	/*
	nstep = MBS_equil.opt.itermax;
	callfct.user.out = str2func(MBS_equil.fctname.user.out);
	callfct.user.out(MBS_data,nstep,'init');
	*/
	// Recherche d'équilibre

	switch (equil_gen_strct->options->solvemethod)
	{
	case 1 :
		{
			MBSdata->process = 2;
			equil_fail = EQUIL_fsolvepk( EQUIL_fct_equil_ptr, mds_gen_strct, part_gen_strct, equil_gen_strct, lds, MBSdata );
			MBSdata->process = 0;
			break;
		}
	case 2 : 
		{
			printf(">>MBS>> mbs_run_equil : Method not yet available - only in Matlab and requires the optimization toolbox\n");
			equil_fail = 1;
			return !equil_fail;
			break;
		}
	}

	if (equil_fail > 0)
	{
		free_double_vec(qd_save);
		return !equil_fail;
	}

	if (equil_gen_strct->options->verbose)
	{
		if (equil_gen_strct->n_x_non_sens >0)
		{
			/*
			list = MBS_equil.xeqname(non_sens_x);
			for i = 1:length(list)-1, list[i]=strcat(list[i],','); end
			printf(">>MBS>> ---> Non sensitive Equilibrium parameters : ',list{1:end}]);
			*/
		}
	}

	/*
	MBS_equil.xeq = x;
	MBS_equil.q = MBSdata->q;    
	MBS_equil.qd = MBSdata->qd;
	MBS_equil.frc = MBSdata->frc;
	MBS_equil.trq = MBSdata->trq;
	MBS_equil.Fl = MBSdata->Fl;
	MBS_equil.lambda = MBSdata->lambda;
	MBS_equil.Qq = MBSdata->Qq;
	*/

	if (equil_gen_strct->options->verbose)
	{
		printf(">>MBS>> ***** mbs_run_equil end *****\n");
	}

	if(equil_gen_strct->options->static_equil)
	{
		copy_double_vec(qd_save, &(MBSdata->qd[1]), mds_gen_strct->bodytree->n_joint);
		free_double_vec(qd_save);
	}
	MBSdata->DoneEquil = 1; 
	equil_fail = 0;
	return !equil_fail;
}


////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Init and free functions for EQUIL structures  /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////


EQUIL_option_strct* init_EQUIL_option_strct(void)
{
	EQUIL_option_strct *equil_option_strct; 

	equil_option_strct = (EQUIL_option_strct*) malloc(sizeof(EQUIL_option_strct));

	equil_option_strct->solvemethod = 1;
	equil_option_strct->relax = 1.0;
	equil_option_strct->smooth = 0;
    equil_option_strct->senstol = 1e-6;
    equil_option_strct->equitol = 1e-6;
	equil_option_strct->devjac = 1e-6;
    equil_option_strct->itermax = 30;
    equil_option_strct->static_equil = 1;
    equil_option_strct->verbose = 1;
    equil_option_strct->visualize = 0;
    equil_option_strct->clearmbsglobal = 1;

	equil_option_strct->n_ignored_qu = 0;
	equil_option_strct->ignored_qu = NULL;

	equil_option_strct->n_changed_qu = 0;
	equil_option_strct->changed_qu = NULL;
	equil_option_strct->x_subst_ptr = NULL;

	equil_option_strct->n_x_extra = 0;
	equil_option_strct->x_extra_ptr = NULL;
	equil_option_strct->EQUIL_extra_fct_ptr = NULL;

	return equil_option_strct;
}
void free_EQUIL_option_strct(EQUIL_option_strct* equil_option_strct)
{
	free_int_vec(equil_option_strct->ignored_qu);
	free_int_vec(equil_option_strct->changed_qu);

	// free x_subst_ptr

	free(equil_option_strct);
}

EQUIL_gen_strct* init_EQUIL_gen_strct(MDS_gen_strct*  mds_gen_strct)
{
	EQUIL_gen_strct *equil_gen_strct; 

	equil_gen_strct = (EQUIL_gen_strct*) malloc(sizeof(EQUIL_gen_strct));

	equil_gen_strct->options = init_EQUIL_option_strct();

	equil_gen_strct->n_x = 0;
	equil_gen_strct->x = NULL;

	equil_gen_strct->n_x_sens = 0;
	equil_gen_strct->x_sens = NULL;

	equil_gen_strct->n_x_non_sens = 0;
	equil_gen_strct->x_non_sens = NULL;

	equil_gen_strct->xeq_ptr = NULL; 
	return equil_gen_strct;
}
void free_EQUIL_gen_strct(EQUIL_gen_strct* equil_gen_strct)
{
	free_EQUIL_option_strct(equil_gen_strct->options);


	free_double_vec(equil_gen_strct->x);
	free_int_vec(equil_gen_strct->x_sens);
	free_int_vec(equil_gen_strct->x_non_sens);

	// free xeq_ptr

	free(equil_gen_strct);
}