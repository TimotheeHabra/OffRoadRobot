#ifndef MDS_TRANLATOR_h
#define MDS_TRANLATOR_h

#include "useful_functions.h"
#include "mbs_xml_reader.h"
#include "MBSdataStruct.h"
#include "MBSfun.h"


MBSdataStruct* MDS_create_MBSdataStruct(MDS_gen_strct* mds_gen_strct);
void MDS_free_MBSdataStruct(MBSdataStruct *s);


#endif
