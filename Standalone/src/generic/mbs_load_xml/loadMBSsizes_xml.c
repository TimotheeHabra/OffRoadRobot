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
 * Code based on loadMBSsizes (in MBSdataStruct.c)
 */
MBSdataStruct* loadMBSsizes_xml(xmlDocPtr doc, xmlNodePtr cur)
{
    MBSdataStruct *s;

	s = (MBSdataStruct*) malloc(sizeof(MBSdataStruct));

    // Parameters
    s->nbody    = parseScalarInteger_xml("nbody", doc, cur);
    s->njoint   = s->nbody; //njoint = nbody
    s->npt      = parseScalarInteger_xml("npt", doc, cur);
    s->nqu      = parseScalarInteger_xml("nqu", doc, cur);
    s->nqc      = parseScalarInteger_xml("nqc", doc, cur);
    s->nqa      = parseScalarInteger_xml("nqa", doc, cur);
    s->nqv      = parseScalarInteger_xml("nqv", doc, cur);
    s->nhu      = parseScalarInteger_xml("nhu", doc, cur);
    s->nqlocked = parseScalarInteger_xml("nqlocked", doc, cur);
    s->nqdriven = parseScalarInteger_xml("nqdriven", doc, cur);
    s->Nloopc   = parseScalarInteger_xml("Nloopc", doc, cur);
    s->Nuserc   = parseScalarInteger_xml("Nuserc", doc, cur);
    s->Ncons    = parseScalarInteger_xml("Ncons", doc, cur);
    s->Nlink    = parseScalarInteger_xml("Nlink", doc, cur);
    s->Nlink3D  = parseScalarInteger_xml("Nlink3D", doc, cur);
    s->Nsensor  = parseScalarInteger_xml("Nsensor", doc, cur);
    s->Nxfrc    = parseScalarInteger_xml("Nxfrc", doc, cur);
    s->Nwheel   = parseScalarInteger_xml("Nwheel", doc, cur);
    #if !defined SENSORKIN
    s->Nuser_model = parseScalarInteger_xml("Nuser_model", doc, cur);
    #endif
    s->Nux = parseScalarInteger_xml("Nux", doc, cur);

    return s;
}
