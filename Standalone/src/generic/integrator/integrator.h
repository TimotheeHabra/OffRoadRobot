/*
 * Integrator headers
 */
#ifndef __INTEGRATOR_H_INCLUDED__  // guard against multiple/recursive includes
#define __INTEGRATOR_H_INCLUDED__

#include "MBSfun.h"
#include "nrutil.h"

void rk4(double y[], double dydx[], int n, double x, double h, double yout[],
         void (*derivs)(double, double [], double [], LocalDataStruct *,MBSdataStruct *),
         LocalDataStruct *lds, MBSdataStruct *s);

void derivs(double x, double y[], double dydx[], LocalDataStruct *lds,MBSdataStruct *s);

#endif
