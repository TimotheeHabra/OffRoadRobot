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
 * Checks the existence of a given element in the XML tree
 */
int isElement_xml(xmlNodePtr a_node, const char* elementName)
{
    int found = 0;
    xmlNodePtr cur_node = NULL;

    for (cur_node = a_node; cur_node; cur_node = cur_node->next) {
        if ((cur_node->type == XML_ELEMENT_NODE) && (!xmlStrcmp(cur_node->name, (const xmlChar *)elementName)))
	    {
		    found = 1;
 	    }

        if(!found)
        {
            found = isElement_xml(cur_node->children, elementName);
        }
    }

    return found;
}

/*
 * Finds a given element in the XML tree and returns a pointer to that element in '*foundNode'
 * Returns 1 if the element is found and 0 otherwise
 */
int findElement_xml(xmlNodePtr a_node, const char* elementName, xmlNodePtr* foundNode)
{
    int found = 0;
    xmlNodePtr cur_node = NULL;

    for (cur_node = a_node; cur_node; cur_node = cur_node->next) {
        if ((cur_node->type == XML_ELEMENT_NODE) && (!xmlStrcmp(cur_node->name, (const xmlChar *)elementName)))
	    {
		    found = 1;
		    *foundNode = cur_node;
 	    }

        if(!found)
        {
            found = findElement_xml(cur_node->children, elementName, foundNode);
        }
    }

    return found;
}

/*
 * Returns the biggest argument
 */
int maxInt(int a, int b)
{
    return (a >= b) ? a : b;
}

/*
 * Populates vectorToExtract with integer values extracted from the string elementValue
 */
void convertVectorInteger(int* vectorToExtract, char* elementValue, int* elementSize)
{
    int i = 0;
    int vectorSize = maxInt(elementSize[0], elementSize[1]);

    // Arrays indexes begin at 1 with the Robotran functions
    vectorToExtract[1] = atoi(strtok(elementValue, " "));

    if(vectorSize > 1)
    {
        for(i=2; i<=vectorSize; i++)
        {
            vectorToExtract[i] = atoi(strtok(NULL, " "));
        }
    }
}

/*
 * Populates vectorToExtract with double values extracted from the string elementValue
 */
void convertVectorDouble(double* vectorToExtract, char* elementValue, int* elementSize)
{
    int i = 0;
    int vectorSize = maxInt(elementSize[0], elementSize[1]);

    // Arrays indexes begin at 1 with the Robotran functions
    vectorToExtract[1] = atof(strtok(elementValue, " "));

    if(vectorSize > 1)
    {
        for(i=2; i<=vectorSize; i++)
        {
            vectorToExtract[i] = atof(strtok(NULL, " "));
        }
    }
}

/*
 * Populates matrixToExtract with double values extracted from the string elementValue
 */
void convertMatrixDouble(double** matrixToExtract, char* elementValue, int* elementSize)
{
    int i,j;

    // Note: the external for loop must iterate on j and the internal one on i
    // so that matrix[i][j] receives the elements in the correct order
    // (i.e. so that (i-1) + nb_row*(j-1) = 0, 1, 2, 3, 4, ...)

    // Note: elementSize = (row, col)

    // Arrays indexes begin at 1 with the Robotran functions
    for(j=1; j<=elementSize[1]; j++)
    {
        for(i=1; i<=elementSize[0]; i++)
        {
            if(i == 1 && j == 1)
            {
                matrixToExtract[i][j] = atof(strtok(elementValue, " "));
            }
            else
            {
                matrixToExtract[i][j] = atof(strtok(NULL, " "));
            }
        }
    }

}

/*
 * Searches for the element "elementName" and parses it as an integer.
 * Note: cur is supposed to be the root element in the case of the mbsdata tree.
 */
int parseScalarInteger_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur)
{
    int scalarToExtract = 0;
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};

    elementValue = parseElement_xml(elementName, doc, cur, elementSize);

    if(elementValue != NULL)
    {
        scalarToExtract = atoi((char*)elementValue);

        // checking results
        //printf("%s (row, col): %d, %d\n", elementName, elementSize[0], elementSize[1]);
        //printf("scalar value extracted: %d\n", scalarToExtract);
    }
    else
    {
        printf("error: element '%s' not found\n", elementName);
    }

    xmlFree(elementValue);

    return scalarToExtract;
}

/*
 * Searches for the element "elementName" and parses it as a double.
 * Note: cur is supposed to be the root element in the case of the mbsdata tree.
 */
double parseScalarDouble_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur)
{
    double scalarToExtract = 0;
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};

    elementValue = parseElement_xml(elementName, doc, cur, elementSize);

    if(elementValue != NULL)
    {
        scalarToExtract = atof((char*)elementValue);

        // checking results
        //printf("%s (row, col): %d, %d\n", elementName, elementSize[0], elementSize[1]);
        //printf("scalar value extracted: %e\n", scalarToExtract);
    }
    else
    {
        printf("error: element '%s' not found\n", elementName);
    }

    xmlFree(elementValue);

    return scalarToExtract;
}

/*
 * Searches for the element "elementName" and parses it as a vector of int
 * Note: cur is supposed to be the root element in the case of the mbsdata tree.
 */
void parseVectorInteger_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, int vectorToExtract[])
{
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};

    elementValue = parseElement_xml(elementName, doc, cur, elementSize);

    if(elementValue != NULL)
    {
        convertVectorInteger(vectorToExtract, (char*)elementValue, elementSize);

        // checking results
        //int i;
        //printf("%s (row, col): %d, %d\n", elementName, elementSize[0], elementSize[1]);
        //for(i=0; i<=maxInt(elementSize[0], elementSize[1]); i++)
        //{
        //    printf("element %d = %d\n", i, vectorToExtract[i]);
        //}
    }
    else
    {
        printf("element '%s' not found\n", elementName);
    }

    xmlFree(elementValue);
}

/*
 * Searches for the element "elementName" and parses it as a vector of double
 * Note: cur is supposed to be the root element in the case of the mbsdata tree.
 */
void parseVectorDouble_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, double vectorToExtract[])
{
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};

    elementValue = parseElement_xml(elementName, doc, cur, elementSize);

    if(elementValue != NULL)
    {
        convertVectorDouble(vectorToExtract, (char*)elementValue, elementSize);

        // checking results
        //int i;
        //printf("%s (row, col): %d, %d\n", elementName, elementSize[0], elementSize[1]);
        //for(i=0; i<=maxInt(elementSize[0], elementSize[1]); i++)
        //{
        //    printf("element %d = %e\n", i, vectorToExtract[i]);
        //}
    }
    else
    {
        printf("element '%s' not found\n", elementName);
    }

    xmlFree(elementValue);
}

/*
 * Searches for the element "elementName" and parses it as a matrix of double
 * Note: cur is supposed to be the root element in the case of the mbsdata tree.
 */
void parseMatrixDouble_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, double** matrixToExtract)
{
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};

    elementValue = parseElement_xml(elementName, doc, cur, elementSize);

    if(elementValue != NULL)
    {
        convertMatrixDouble(matrixToExtract, (char*)elementValue, elementSize);

        // checking results
        //int i, j;
        //printf("%s (row, col): %d, %d\n", elementName, elementSize[0], elementSize[1]);
        //for(i=1; i<=elementSize[0]; i++)
        //{
        //    for(j=1; j<=elementSize[1]; j++)
        //    {
        //        printf("%e\t", matrixToExtract[i][j]);
        //    }
        //    printf("\n");
        //}
    }
    else
    {
        printf("element '%s' not found\n", elementName);
    }

    xmlFree(elementValue);
}

/*
 * Walks through the XML tree searching for the element "elementName".
 * When elementName is found, its size (int tab[2]) is extracted and parsed.
 * Finally, the number of columns of the elements (size[1]) is returned.
 */
int getNbColElement_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur)
{
    int elementSize[2] = {0,0};

    cur = cur->xmlChildrenNode;

	while (cur != NULL) {
	    if ((!xmlStrcmp(cur->name, (const xmlChar *)elementName)))
	    {
		    getSize_xml(cur, elementSize); // element size extraction and parsing
 	    }
        cur = cur->next;
	}

	return elementSize[1];
}

/*
 * Walks through the XML tree searching for the element "elementName".
 * When elementName is found, its size (int tab[2]) and value (xmlChar*) are extracted.
 * Note: xmlChar *elementValue is dynamically allocated inside this function and must therefore
 * be freed using xmlFree(elementValue) (do not forget !).
 */
xmlChar* parseElement_xml(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, int elementSize[2])
{
	xmlChar* elementValue = NULL;
	cur = cur->xmlChildrenNode;

	while (cur != NULL) {
	    if ((!xmlStrcmp(cur->name, (const xmlChar *)elementName)))
	    {
		    elementValue = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1); // element value extraction
		    getSize_xml(cur, elementSize); // element size extraction and parsing
 	    }
        cur = cur->next;
	}

	return elementValue;
}

/*
 * Get the "size" attribute of an XML element of the MBSdata structure
 * and parse it into a set of integers.
 */
void getSize_xml(xmlNodePtr cur, int tab[2])
{
    xmlChar *uri;

    uri = xmlGetProp(cur, (const xmlChar *)"size");
    //printf("size: %s\t", uri);

    // Parse size
    tab[0] = atoi(strtok((char*)uri, " "));
    tab[1] = atoi(strtok(NULL, " "));

    //printf(" --- %d, %d\t", tab[0], tab[1]);
    xmlFree(uri);

	return;
}

/*
 * print_element_names:
 * @a_node: the initial xml node to consider.
 *
 * Prints the names of the all the xml elements
 * that are siblings or children of a given xml node.
 */
void print_element_names_xml(xmlNodePtr a_node)
{
    xmlNodePtr cur_node = NULL;
    int tab[2];

    for (cur_node = a_node; cur_node; cur_node = cur_node->next) {
        if (cur_node->type == XML_ELEMENT_NODE) {
            printf("node type: Element, name: %s\t", cur_node->name);
            getSize_xml(cur_node, tab);
            printf("\n");
        }

        print_element_names_xml(cur_node->children);
    }
}
