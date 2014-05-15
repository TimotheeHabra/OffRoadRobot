/*
 * Functions to initialize the MBSdata structure from the <ProjectName>.mbsdata (xml format) file.
 *
 * Allan Barrea Feb. 2013
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include "user_sf_IO.h"
#include "MBSfun.h"
#include "sfdef.h"
#include "MBSdef.h"

UserModelStruct* loadUserModel_xml(const xmlDocPtr doc, const xmlNodePtr cur)
{
    UserModelStruct *ums = NULL;

    return ums;
}
