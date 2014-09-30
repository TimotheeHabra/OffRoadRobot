#ifndef __LOAD_MBSXML_H_INCLUDED__  // guard against multiple/recursive includes
#define __LOAD_MBSXML_H_INCLUDED__

#include "MBSdataStruct.h"
#include "mds_translator.h"
#include "part.h"

MBSdataStruct* loadMBSdata_xml(const char *filein);

#endif
