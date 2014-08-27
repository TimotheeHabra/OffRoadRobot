
#ifndef PART_h
#define PART_h

#include "useful_functions.h"
#include "mbs_xml_reader.h"
#include "MBSdataStruct.h"
#include "MBSfun.h"

typedef struct PART_option_strct
{
	int rowperm; // no = 0, yes = 1, defaut = 0
	int visualise; // no = 0, yes = 1, defaut = 0
	double treshold; // defaut = 1e-9
	int drivers; // no = 0, yes = 1, defaut = 0
	int verbose; // no = 0, yes = 1, defaut = 1
	int clearmbsglobal; // inout = 1, out = 2, none = 3, all = 4, defaut = 1

} PART_option_strct;

typedef struct PART_gen_strct
{
	PART_option_strct *options; 

	int n_qu; 
	int *ind_qu;

	int n_qv; 
	int *ind_qv; 

	int n_hu; 
	int *ind_hu; 

	int n_hv; 
	int *ind_hv; 

	double *q_closed;

} PART_gen_strct;

int PART_run_part(MDS_gen_strct*  mds_gen_strct, MBSdataStruct* s, PART_gen_strct*  part_gen_strct);

int rank_double_tab(double** matrix, int x, int y); // change the place of this

PART_option_strct* init_PART_option_strct(void);
void free_PART_option_strct(PART_option_strct* part_option_strct);

PART_gen_strct* init_PART_gen_strct(MDS_gen_strct*  mds_gen_strct);
void free_PART_gen_strct(PART_gen_strct* part_gen_strct);

void PART_get_options_from_user(PART_option_strct *options);

#endif