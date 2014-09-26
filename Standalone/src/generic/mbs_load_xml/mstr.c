
#include "mstr.h"


MSTR_strct* init_MSTR_strct(void)
{
	MSTR_strct *mstr_strct; 

	mstr_strct = (MSTR_strct*) malloc(sizeof(MSTR_strct));

	mstr_strct->mds = NULL;
	mstr_strct->part = NULL;
    //mstr_strct->equil = NULL;
    //mstr_strct->lds = NULL;
	mstr_strct->MBSdata = NULL;

	return mstr_strct;
}
void free_MSTR_strct(MSTR_strct *mstr_strct)
{
	free_MDS_gen_strct(mstr_strct->mds);
	free_PART_gen_strct(mstr_strct->part);
    //free_EQUIL_gen_strct(mstr_strct->equil);

    //freeLocalDataStruct(mstr_strct->lds, mstr_strct->MBSdata); // change it
	MDS_free_MBSdataStruct(mstr_strct->MBSdata);

	free(mstr_strct);
}

void MSTR_exe_load(MSTR_strct *mstr_strct, char* mbs_xml_name)
{
	mstr_strct->mds = MDS_mbs_reader(mbs_xml_name);
	mstr_strct->MBSdata = MDS_create_MBSdataStruct(mstr_strct->mds); // pourrai etre void 
    //mstr_strct->lds = initLocalDataStruct(mstr_strct->MBSdata);
}
void MSTR_exe_part(MSTR_strct *mstr_strct)
{
	mstr_strct->part = init_PART_gen_strct(mstr_strct->mds);
	PART_get_options_from_user(mstr_strct->part->options);
	PART_run_part(mstr_strct->mds, mstr_strct->MBSdata, mstr_strct->part);
}
//void MSTR_exe_equil(MSTR_strct *mstr_strct)
//{
//	/*double *f;
//	f = get_double_vec(1);*/

//	mstr_strct->equil = init_EQUIL_gen_strct(mstr_strct->mds);
//	EQUIL_get_options_from_user(mstr_strct->equil->options, mstr_strct->MBSdata);

//	/*(*mstr_strct->equil->options->EQUIL_extra_fct_ptr)(mstr_strct->MBSdata,f);
//	printf("%f",f[0]);*/

///*
//	freeLocalDataStruct(mstr_strct->lds,mstr_strct->MBSdata);
//	mstr_strct->lds = initLocalDataStruct(mstr_strct->MBSdata);
//	*/
	
//	EQUIL_run_equil(mstr_strct->mds, mstr_strct->part, mstr_strct->equil, mstr_strct->lds, mstr_strct->MBSdata);
//}

void MSTR_exe_close(MSTR_strct *mstr_strct)
{
	
}
