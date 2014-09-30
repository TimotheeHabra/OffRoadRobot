
#include "loadMBSdata_xml.h"

MBSdataStruct* loadMBSdata_xml(const char *mbs_xml_name)
{
    MDS_gen_strct *mds      = NULL;
    MBSdataStruct *MBSdata  = NULL;
    PART_gen_strct *part    = NULL;

    // retrieve data from xml .mbs
    mds = MDS_mbs_reader(mbs_xml_name);

    //load mbsdata struct
    MBSdata = MDS_create_MBSdataStruct(mds);

    // coordinate partitioning
    part = init_PART_gen_strct(mds);

    part->options->rowperm = 1;
    //part->options->visualise = 0;
    //part->options->treshold = 1e-9;
    //part->options->drivers = 0;
    //part->options->verbose = 0;
    //part->options->clearmbsglobal = 1;

    PART_run_part(mds, MBSdata, part);

    free_MDS_gen_strct(mds);
    free_PART_gen_strct(part);

    return MBSdata;
}
