//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------
//
// Gestion via Bugzilla :
// 01/10/2008 : JFC : Bug n°36
//

#ifndef lut_h
#define lut_h
/*--------------------*/

//#ifndef CMEX
//#include "simstruc.h"
//#else
//#include "mex.h"
//#endif

typedef struct LUT_2D
{
	int nx;
	int ny;
	double *x;
	double *y;
	double **z;
} LUT_2D;


typedef struct LUT_1D
{
	int nx;
	double *x;
	double *y;
} LUT_1D;


LUT_1D* alloc_LUT_1D();

LUT_2D* alloc_LUT_2D();


void free_LUT_1D(LUT_1D *lut);

void free_LUT_2D(LUT_2D *lut);


double polin1(LUT_1D *lut, double x);

double polin2(LUT_2D *lut, double x, double y);

#ifdef MATLAB_MEX_FILE

LUT_1D* load_LUT_1D(const mxArray *LUT_1D_ptr);

LUT_2D* load_LUT_2D(const mxArray *LUT_2D_ptr);

#endif

LUT_1D* load_LUT_1D_2inputs(const double *LUT_1D_ptr, const int nx);

LUT_1D* load_LUT_1D_3inputs(const double *x_ptr,const double *y_ptr,const int nx);

/*--------------------*/
#endif
