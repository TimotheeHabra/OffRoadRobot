
#ifndef EQUIL_h
#define EQUIL_h

#include "useful_functions.h"

#include "mbs_xml_reader.h"
#include "part.h"
#include "MBSdataStruct.h"
#include "MBSfun.h"

//#include "mstr.h"

typedef struct EQUIL_option_strct
{
	int solvemethod; // fsolvepk = 1, optim = 2, defaut = 1
	double relax; // defaut = 1.0
	int smooth; // no = 0, yes = 1, defaut = 0
    double senstol; // defaut = 1e-6
    double equitol; // defaut = 1e-6
	double devjac; //default = 1e-6
    int itermax; // defaut = 30
    int static_equil; // no = 0, yes = 1, defaut = 1 => all velocities are reset to zero
    int verbose; // no = 0, yes = 1, defaut = 1
    int visualize; // no = 0, yes = 1, defaut = 0
    int clearmbsglobal; // inout = 1, out = 2, none = 3, all = 4, defaut = 1

	int n_ignored_qu;
	int *ignored_qu;
	
	int n_changed_qu;
	int *changed_qu;	
	double **x_subst_ptr;

	int n_x_extra; // faire les init et free
	double **x_extra_ptr;
	int (* EQUIL_extra_fct_ptr)(MBSdataStruct*, double*); 

} EQUIL_option_strct;

typedef struct EQUIL_gen_strct
{
	EQUIL_option_strct *options; 

	int n_x;
	double *x;

	int n_x_sens;
	int *x_sens;

	int n_x_non_sens;
	int *x_non_sens;

	double **xeq_ptr;

} EQUIL_gen_strct;


void ludcmp_2(double **a, int n, int *indx, double *d);
void lubksb_2(double **a, int n, int *indx, double b[]);


int compute_Fr_uc( double *Fr_uc, MDS_gen_strct *mds_gen_strct, PART_gen_strct *part_gen_strct,	LocalDataStruct *lds, MBSdataStruct *MBSdata );

int EQUIL_run_equil(MDS_gen_strct *mds_gen_strct, PART_gen_strct *part_gen_strct, EQUIL_gen_strct *equil_gen_strct,	LocalDataStruct *lds, MBSdataStruct *MBSdata);


EQUIL_option_strct* init_EQUIL_option_strct(void);
void free_EQUIL_option_strct(EQUIL_option_strct* equil_option_strct);

EQUIL_gen_strct* init_EQUIL_gen_strct(MDS_gen_strct*  mds_gen_strct);
void free_EQUIL_gen_strct(EQUIL_gen_strct* equil_gen_strct);

void EQUIL_get_options_from_user(EQUIL_option_strct *options, MBSdataStruct *MBSdata);

#endif