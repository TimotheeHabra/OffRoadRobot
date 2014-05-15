/*
 * Function related to the integrator (Runge Kutta order 4, from Numerical Recipes)
 */

#include "integrator.h"

/*
 * Given values for the variables y[1..n] and their derivatives dydx[1..n] known at x, use the
 * fourth-order Runge-Kutta method to advance the solution over an interval h and return the
 * incremented variables as yout[1..n], which need not be a distinct array from y. The user
 * supplies the routine derivs(x,y,dydx) , which returns derivatives dydx at x.
 */
void rk4(double y[], double dydx[], int n, double x, double h, double yout[],
         void (*derivs)(double, double [], double [], LocalDataStruct *,MBSdataStruct *),
         LocalDataStruct *lds, MBSdataStruct *s)
{
    int i;
    double xh,hh,h6,*dym,*dyt,*yt;

    dym=dvector(1,n);
    dyt=dvector(1,n);
    yt=dvector(1,n);

    hh=h*0.5;
    h6=h/6.0;

    xh=x+hh;

    // First step (derivs already called once in odeint)
    for (i=1;i<=n;i++)
        yt[i]=y[i]+hh*dydx[i];

    // Second step
    (*derivs)(xh,yt,dyt,lds,s);
    for (i=1;i<=n;i++)
        yt[i]=y[i]+hh*dyt[i];

    // Third step
    (*derivs)(xh,yt,dym,lds,s);
    for (i=1;i<=n;i++)
    {
        yt[i]=y[i]+h*dym[i];
        dym[i] += dyt[i];
    }

    // Fourth step
    (*derivs)(x+h,yt,dyt,lds,s);

    // Accumulate increments with proper weights
    for (i=1;i<=n;i++)
        yout[i]=y[i]+h6*(dydx[i]+dyt[i]+2.0*dym[i]);

    free_dvector(yt,1,n);
    free_dvector(dyt,1,n);
    free_dvector(dym,1,n);
}
