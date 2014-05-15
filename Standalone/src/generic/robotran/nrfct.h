//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#ifndef mbs_matrix_h
#define mbs_matrix_h
/*--------------------*/

void ludcmp(double **a, int n, int *indx, double *d);
void lubksb(double **a, int n, int *indx, double b[]);
void choldc(double **a, int n, double p[]);
void cholsl(double **a, int n, double p[], double b[], double x[]);
void svdcmp(double **a, int m, int n, double w[], double **v);
void svbksb(double **u, double w[], double **v, int m, int n, double b[], double x[]);
void gaussj(double **a, int n, double **b, int m);

#endif
