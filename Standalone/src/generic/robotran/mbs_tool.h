//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#ifndef mbs_mxtool_h
#define mbs_mxtool_h
/*--------------------*/

void put_vector(double *v, int size, char *name);

void put_ivector(int *v, int size, char *name);

void put_matrix(double **m, int nL, int nC, char *name);

double *mbs_vector(int n);
void free_mbs_vector(double *v);

int *mbs_ivector(int n);
void free_mbs_ivector(int *v);

double **mbs_matrix(int nr, int nc);
void free_mbs_matrix(double **m, int nr);


/*--------------------*/
#endif
