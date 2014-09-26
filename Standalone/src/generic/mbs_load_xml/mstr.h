
#ifndef MSTR_h
#define MSTR_h

#include "useful_functions.h"

#include "mbs_xml_reader.h"
#include "mds_translator.h"
#include "part.h"
//#include "equil.h"
//#include "modal.h"

#include "MBSfun.h"


typedef struct MSTR_strct
{
	MDS_gen_strct *mds;
	PART_gen_strct *part;
    //EQUIL_gen_strct *equil;
	//MODAL_gen_strct *modal;
    //LocalDataStruct *lds;
	MBSdataStruct *MBSdata;

} MSTR_strct;

MSTR_strct* init_MSTR_strct(void);
void free_MSTR_strct(MSTR_strct *mstr_strct);

void MSTR_exe_load(MSTR_strct *mstr_strct,char* mbs_xml_name); 
void MSTR_exe_part(MSTR_strct *mstr_strct); 
void MSTR_exe_equil(MSTR_strct *mstr_strct);
//void MSTR_exe_modal(MSTR_strct *mstr_strct);
// put here other modules
void MSTR_exe_close(MSTR_strct *mstr_strct); 


#endif
