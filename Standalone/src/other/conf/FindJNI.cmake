#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# This file finds the include folders (header files) and the libraries
# related to JNI (Java Native Interface)
#
# JNI_FOUND :        1 if all required files found (0 otherwise)
# JNI_INCLUDE_PATH : include paths (for the header files) -> for compilation
# JNI_LIBRARIES :    libraries -> for linkage
#


IF (UNIX)

IF (APPLE)
	

## ---- MAC OS ---- ##

# possible paths for: 'jni.h'
SET(TRIAL_PATHS_INC_JNI
  /Library/Java/JavaVirtualMachines/jdk1.7.0_45.jdk/Contents/Home/include
) 

# possible paths for: 'jni_md.h'
SET(TRIAL_PATHS_INC_JNI_MD
  /Library/Java/JavaVirtualMachines/jdk1.7.0_45.jdk/Contents/Home/include/darwin
) 

# possible paths for: 'libjvm.dylib'
SET(TRIAL_PATHS_LIB
  /Library/Java/JavaVirtualMachines/jdk1.7.0_45.jdk/Contents/Home/jre/lib/server
) 

FIND_PATH(JNI_INCLUDE_JNI jni.h ${TRIAL_PATHS_INC_JNI})
FIND_PATH(JNI_INCLUDE_JNI_MD jni_md.h ${TRIAL_PATHS_INC_JNI_MD})

FIND_LIBRARY(JNI_LIBRARIES libjvm.dylib ${TRIAL_PATHS_LIB})

SET(JNI_INCLUDE_PATH ${JNI_INCLUDE_JNI} ${JNI_INCLUDE_JNI_MD})

## ---------------- ##


ELSE (APPLE)
	

## ---- LINUX ---- ##	

# possible paths for: 'jni.h'
SET(TRIAL_PATHS_INC_JNI
  /usr/lib/jvm/java-7-openjdk-amd64/include
  /usr/lib/jvm/java-6-sun/include

  /usr/lib/jvm/java-7-openjdk-i386/include
) 

# possible paths for: 'jni_md.h'
SET(TRIAL_PATHS_INC_JNI_MD
  /usr/lib/jvm/java-7-openjdk-amd64/include/linux
  /usr/lib/jvm/java-6-sun/include/linux

  /usr/lib/jvm/java-7-openjdk-i386/include/linux
) 

# possible paths for: 'libjvm.so'
SET(TRIAL_PATHS_LIB
  /usr/lib/jvm/java-7-openjdk-amd64/jre/lib/amd64/server
  /usr/lib/jvm/java-6-sun/jre/lib/amd64/server

  /usr/lib/jvm/java-7-openjdk-i386/jre/lib/i386/server
) 

FIND_PATH(JNI_INCLUDE_JNI jni.h ${TRIAL_PATHS_INC_JNI})
FIND_PATH(JNI_INCLUDE_JNI_MD jni_md.h ${TRIAL_PATHS_INC_JNI_MD})

FIND_LIBRARY(JNI_LIBRARIES libjvm.so ${TRIAL_PATHS_LIB})

SET(JNI_INCLUDE_PATH ${JNI_INCLUDE_JNI} ${JNI_INCLUDE_JNI_MD})

## --------------- ##


ENDIF (APPLE)

ELSE (UNIX)


## ---- WINDOWS ---- ##

# possible paths for: 'jni.h'
SET(TRIAL_PATHS_INC_JNI
  C:/Program\ Files/Java/jdk1.7.0_51/include
  C:/Program\ Files/Java/jdk1.8.0/include
) 

# possible paths for: 'jni_md.h'
SET(TRIAL_PATHS_INC_JNI_MD
  C:/Program\ Files/Java/jdk1.7.0_51/include/win32
  C:/Program\ Files/Java/jdk1.8.0/include/win32
) 

# possible paths for: 'jvm.lib'
SET(TRIAL_PATHS_LIB
  C:/Program\ Files/Java/jdk1.7.0_51/lib
  C:/Program\ Files/Java/jdk1.8.0/lib
) 

FIND_PATH(JNI_INCLUDE_JNI jni.h ${TRIAL_PATHS_INC_JNI})
FIND_PATH(JNI_INCLUDE_JNI_MD jni_md.h ${TRIAL_PATHS_INC_JNI_MD})

FIND_LIBRARY(JNI_LIBRARIES jvm.lib ${TRIAL_PATHS_LIB})

SET(JNI_INCLUDE_PATH ${JNI_INCLUDE_JNI} ${JNI_INCLUDE_JNI_MD})

## ----------------- ##


ENDIF (UNIX)


# flag put to 1 if all required files are found
IF(JNI_INCLUDE_PATH AND JNI_LIBRARIES)
  SET(JNI_FOUND 1)
ELSE(JNI_INCLUDE_PATH AND JNI_LIBRARIES)
  SET(JNI_FOUND 0)
ENDIF(JNI_INCLUDE_PATH AND JNI_LIBRARIES)

# outputs
MARK_AS_ADVANCED(
  JNI_FOUND
  JNI_INCLUDE_PATH
  JNI_LIBRARIES 
)
