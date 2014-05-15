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

#include "math.h"
#include "lut.h"
#include "MBSfun.h"

LUT_1D* alloc_LUT_1D()
{
	LUT_1D* newlut;
	newlut=(LUT_1D*) malloc(sizeof(LUT_1D));
	return newlut;
}

LUT_2D* alloc_LUT_2D()
{
	LUT_2D* newlut;
	newlut=(LUT_2D*) malloc(sizeof(LUT_2D));
	return newlut;
}

void free_LUT_1D(LUT_1D *lut)
{
	free(lut->x);
	free(lut->y);

	free(lut);
}

void free_LUT_2D(LUT_2D *lut)
{
	int i;

	free(lut->x);
	free(lut->y);
	
	for(i=1;i<=lut->nx;i++)
	{
		free(lut->z[i]);
	}
	free(lut->z);

	free(lut);
}

/*****************************************************************************/

double polin2(LUT_2D *lut, double x, double y)
{
	int indx,indy;
	int step,min,max;
	double alpha,beta,gamma,zeta;
	double z;
	

	min=1;
	max=lut->nx;
	while(max-min>1)
	{
		step=(max-min)/2;
		if(x-lut->x[min+step] >=0)
		{
			min+=step;
		}
		else
		{
			max-=step;
		}
	}
	indx=min;


	if(indx>=lut->nx) 
		indx=lut->nx-1;

	if(indx != min)
	{
		min=(min+min)/2;
	}

/**/	
	min=1;
	max=lut->ny;
	while(max-min>1)
	{
		step=(max-min)/2;
		if(y-lut->y[min+step] >=0)
		{
			min+=step;
		}
		else
		{
			max-=step;
		}
	}
	indy=min;

	if(indy >= lut->ny) 
		indy=lut->ny-1;

	if(indy != min)
	{
		min=(min+min)/2;
	}

	/**/

	alpha = lut->z[indx][indy]/((lut->x[indx+1]-lut->x[indx])*(lut->y[indy+1]-lut->y[indy]));
	beta = lut->z[indx+1][indy+1]/((lut->x[indx+1]-lut->x[indx])*(lut->y[indy+1]-lut->y[indy]));
	gamma = lut->z[indx+1][indy]/((lut->x[indx+1]-lut->x[indx])*(lut->y[indy+1]-lut->y[indy]));
	zeta = lut->z[indx][indy+1]/((lut->x[indx+1]-lut->x[indx])*(lut->y[indy+1]-lut->y[indy]));
	
	z = alpha*((lut->x[indx+1]-x)*(lut->y[indy+1]-y))+beta*((x-lut->x[indx])*(y-lut->y[indy]))+gamma*((x-lut->x[indx])*(lut->y[indy+1]-y))+zeta*((lut->x[indx+1]-x)*(y-lut->y[indy]));
	
	return z;
}

/*****************************************************************************/


double polin1(LUT_1D *lut, double x)
{
	int indx;
	int step,min,max;
	double alpha;
	double y;
	

	min=1;
	max=lut->nx;
	while(max-min>1)
	{
		step=(max-min)/2;
		if(x-lut->x[min+step] >=0)
		{
			min+=step;
		}
		else
		{
			max-=step;
		}
	}
	indx=min;


	if(indx>=lut->nx) 
		indx=lut->nx-1;

	if(indx != min)
	{
		min=(min+min)/2;
	}


	/**/

	alpha = (lut->y[indx+1]-lut->y[indx])/(lut->x[indx+1]-lut->x[indx]);
	
	y = alpha*(x-lut->x[indx])+lut->y[indx];
	
	return y;
}

/*****************************************************************************/

#ifdef MATLAB_MEX_FILE
LUT_1D* load_LUT_1D(const mxArray *LUT_1D_ptr)
{
	mxArray *field_value_ptr;
	int i;
	
	LUT_1D* lut;

	lut = alloc_LUT_1D();
	// x //
	field_value_ptr = mxGetField(LUT_1D_ptr, 0,  "x");
	lut->nx = mxGetM(field_value_ptr);
	lut->x = (double*) calloc(lut->nx +1,sizeof(double));
	for(i=1;i<=lut->nx ;i++) 
	{
		lut->x[i] = mxGetPr(field_value_ptr)[i-1];
	}

	// y //
	field_value_ptr = mxGetField(LUT_1D_ptr, 0,  "y");
	lut->y = (double*) calloc(lut->nx +1,sizeof(double));
	for(i=1;i<=lut->nx ;i++) 
	{
		lut->y[i] = mxGetPr(field_value_ptr)[i-1];
	}

	return lut;
}


/*****************************************************************************/


LUT_2D* load_LUT_2D(const mxArray *LUT_2D_ptr)
{
	mxArray *field_value_ptr;
	int i,j;

	LUT_2D* lut;

	lut = alloc_LUT_2D();
	// x //
	field_value_ptr = mxGetField(LUT_2D_ptr, 0,  "x");
	lut->nx = mxGetM(field_value_ptr);
	lut->x = (double*) calloc(lut->nx +1,sizeof(double));
	for(i=1;i<=lut->nx ;i++) 
	{
		lut->x[i] = mxGetPr(field_value_ptr)[i-1];
	}

	// y //
	field_value_ptr = mxGetField(LUT_2D_ptr, 0,  "y");
	lut->ny = mxGetM(field_value_ptr);
	lut->y = (double*) calloc(lut->ny +1,sizeof(double));
	for(i=1;i<=lut->ny ;i++) 
	{
		lut->y[i] = mxGetPr(field_value_ptr)[i-1];
	}

	// z //
	field_value_ptr = mxGetField(LUT_2D_ptr, 0,  "z");
	lut->z = (double**) calloc(lut->nx +1,sizeof(double*));
	for(i=1;i<=lut->nx ;i++) 
	{
		lut->z[i] = (double*) calloc(lut->ny+1,sizeof(double)); 
		for(j=1;j<=lut->ny;j++)
		{
			lut->z[i][j] = mxGetPr(field_value_ptr)[(i-1)+lut->nx*(j-1)];
		}
	}

	return lut;

}
#endif

/*****************************************************************************/

LUT_1D* load_LUT_1D_2inputs(const double *LUT_1D_ptr,const int nx)
{
	int i;
	
	LUT_1D* lut;

	lut = alloc_LUT_1D();
	// x //
	lut->nx = nx;
	lut->x = (double*) calloc(lut->nx +1,sizeof(double));
	for(i=1;i<=lut->nx ;i++) 
	{
		lut->x[i] = LUT_1D_ptr[i-1];
	}

	// y //
	lut->y = (double*) calloc(lut->nx +1,sizeof(double));
	for(i=1;i<=lut->nx ;i++) 
	{
		lut->y[i] = LUT_1D_ptr[nx+i-1];
	}

	return lut;
}


LUT_1D* load_LUT_1D_3inputs(const double *x_ptr,const double *y_ptr,const int nx)
{
	int i;
	
	LUT_1D* lut;

	lut = alloc_LUT_1D();
	// x //
	lut->nx = nx;
	lut->x = (double*) calloc(lut->nx +1,sizeof(double));
	for(i=1;i<=lut->nx ;i++) 
	{
		lut->x[i] = x_ptr[i-1];
	}

	// y //
	lut->y = (double*) calloc(lut->nx +1,sizeof(double));
	for(i=1;i<=lut->nx ;i++) 
	{
		lut->y[i] = y_ptr[i-1];
	}

	return lut;
}
