/*
 * Header file for all functions related to loadMBS_xml
 *
 * Allan Barrea Feb. 2013
 */
#ifndef __LOOP_SIMULATION_H_INCLUDED__  // guard against multiple/recursive includes
#define __LOOP_SIMULATION_H_INCLUDED__

#include "MBSfun.h"


MBSdataStruct* loadMBSdata_xml(const char *filein);
MBSdataStruct* loadMBSsizes_xml(xmlDocPtr doc, xmlNodePtr cur);
void loadMBSelements_xml(MBSdataStruct *MBSdata, xmlDocPtr doc, xmlNodePtr cur);

UserModelStruct* loadUserModel_xml(const xmlDocPtr doc, const xmlNodePtr cur);

int isElement_xml(xmlNodePtr a_node, const char* elementName);
int findElement_xml(xmlNodePtr a_node, const char* elementName, xmlNodePtr* foundNode);
int maxInt(int a, int b);

void convertVectorInteger(int* vectorToExtract, char* elementValue, int* elementSize);
void convertVectorDouble(double* vectorToExtract, char* elementValue, int* elementSize);
void convertMatrixDouble(double** matrixToExtract, char* elementValue, int* elementSize);

int parseScalarInteger_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur);
double parseScalarDouble_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur);
void parseVectorInteger_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, int vectorToExtract[]);
void parseVectorDouble_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, double vectorToExtract[]);
void parseMatrixDouble_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, double** matrixToExtract);

int getNbColElement_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur);
xmlChar* parseElement_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, int elementSize[2]);
void getSize_xml(xmlNodePtr cur, int tab[2]);
void print_element_names_xml(xmlNodePtr a_node);

void freeMBSdata_xml(MBSdataStruct *s);
void freeUserModel_xml(UserModelStruct *ums);

#endif
