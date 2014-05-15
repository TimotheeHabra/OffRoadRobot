/*
 * JNI (java virtual machine so as to access to the 3D animation) main header
 *
 * authors: Nicolas Van der Noot and Nicolas Docquier
 */
#ifndef __JNI_FUNCTIONS_H_INCLUDED__  // guard against multiple/recursive includes
#define __JNI_FUNCTIONS_H_INCLUDED__

#if defined(JNI) & defined(REAL_TIME)

#include <jni.h>

#include "MBSfun.h"
#include "real_time.h"
#include "cmake_config.h"
#include "project_info.h"

// version of the JDK
#define JNI_VERSION JNI_VERSION_1_6

// path to the .mbs file (usually located in the dataR folder) -> provide the correct name
#define MBS_FILE PROJECT_ABS_PATH"/../dataR/"PROJECT_NAME_MBS 

// path to the jar file containing mbsyspad bynary code -> normally no changes required
#define JAR_PATH PROJECT_ABS_PATH"/src/other/mbsyspad/MBsysPad.jar" 

// path to the folder containing additional libraries, in particular the java3D native file (libj3dcore-ogl.so for linux)
// -> fill with the correct path on your computer
#define JAVA3D_PATH J3D_ABS_PATH

// -- Structures -- //

// JNI strcuture
typedef struct JNI_struct
{
    JNIEnv* env;
    jmethodID updateJointMethod;
    jobject obj;

    jmethodID setVpMethod;
    jobject model3D;
    
} JNI_struct;

// create a JNI structure
JNIEnv* create_vm(void);

// initialization
JNI_struct* init_jni(MBSdataStruct *MBSdata);

// update
void update_jni(JNI_struct *jni_struct, MBSdataStruct *s, Simu_real_time *real_time);

// close operations
void free_jni(JNI_struct *jni_struct);

#endif

#endif
