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

/*
 * Frees the memory allocated to the UserModel structure.
 */
void freeUserModel_xml(UserModelStruct *ums)
{

}
