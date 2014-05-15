/*
 * Function related to the integrator (Runge Kutta order 4, from Numerical Recipes)
 */

#include "integrator.h"

/*
 * Computes the derivatives of the system to integrate
 * x = t (i.e. "t" is denoted by "x" here = independent variable)
 */
void derivs(double x, double y[], double dydx[], LocalDataStruct *lds,MBSdataStruct *s)
{
	int i;

	s->tsim = x;

	// Update state variables
	for(i=1;i<=s->nqu;i++)
	{
		s->q[s->qu[i]] = y[i];
		s->qd[s->qu[i]] = y[i+s->nqu];
	}
	for(i=1;i<=s->Nux;i++)
	{
		s->ux[i] = y[i+2*s->nqu];
	}

	// Direct Dynamics computation (or with Accelred)

	#ifdef DIRDYNARED
	i = dirdynared(lds,s);
	#elif defined ACCELRED
	i = accelred(s->qddu,s,s->tsim);
	#endif

	if(i<0) printf("Loop closing Error : NR iteration overrun !\n");

	// User Derivatives
	if(s->Nux>0) user_Derivative(s);

	// Update state vector
	for(i=1;i<=s->nqu;i++)
	{
		dydx[i] = s->qd[s->qu[i]];
		dydx[i+s->nqu] = s->qddu[i];
	}
	for(i=1;i<=s->Nux;i++)
	{
		dydx[i+2*s->nqu] = s->uxd[i];
	}
}
