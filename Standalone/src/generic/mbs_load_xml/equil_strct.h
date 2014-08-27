
#ifndef EQUIL_h
#define EQUIL_h

#include "useful_functions.h"

#include "mbs_xml_reader.h"
#include "MBSdataStruct.h"

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
    int xeqchoice; //',xeqvalue},                  % defaut = les qu
    int static_v; // no = 0, yes = 1, defaut = 1 => all velocities are reset to zero
    int verbose; // no = 0, yes = 1, defaut = 1
    int visualize; // no = 0, yes = 1, defaut = 0
    int clearmbsglobal; // inout = 1, out = 2, none = 3, all = 4, defaut = 1

} EQUIL_option_strct;

typedef struct EQUIL_gen_strct
{
	EQUIL_option_strct *options; 

	double **xeq_ptr;

} EQUIL_gen_strct;



int EQUIL_run_equil(MDS_gen_strct*  mds_gen_strct);


EQUIL_option_strct* init_EQUIL_option_strct(void);
void free_EQUIL_option_strct(EQUIL_option_strct* equil_option_strct);

EQUIL_gen_strct* init_EQUIL_gen_strct(MDS_gen_strct*  mds_gen_strct);
void free_EQUIL_gen_strct(EQUIL_gen_strct* equil_gen_strct);

#endif