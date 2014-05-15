/*===========================================================================*
  *
  *  UserModelStruct.c
  *	
  *  (c) Universite catholique de Louvain
  *      Département de Mécanique 
  *      Unité de Production Mécanique et Machines 
  *      2, Place du Levant 
  *      1348 Louvain-la-Neuve 
  *  http://www.robotran.be// 
  *  
 /*===========================================================================*/

#include "MBSdataStruct.h"

#include "lut.h"

#ifndef CMEX
UserModelStruct * loadUserModel(mxArray *usm_ptr)
#else
UserModelStruct * loadUserModel()
#endif
{
	return NULL;
}

void freeUserModel(UserModelStruct *ums)
{

}
