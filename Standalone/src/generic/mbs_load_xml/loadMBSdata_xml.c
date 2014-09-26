
#include "loadMBSdata_xml.h"

MBSdataStruct* loadMBSdata_xml(const char *mbs_xml_name)
{
    MDS_gen_strct *mds      = NULL;
    MBSdataStruct *MBSdata  = NULL;

    mds = MDS_mbs_reader(mbs_xml_name);
    MBSdata = MDS_create_MBSdataStruct(mds);
    free_MDS_gen_strct(mds);

    return MBSdata;
}
