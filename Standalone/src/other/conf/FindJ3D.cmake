#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# This file finds the libraries
# related to Java 3D
#
# J3D_FOUND : 1 if all required files found (0 otherwise)
# J3D_PATH  : libraries -> for linkage or during the execution (Windows)
#


IF (UNIX)

IF (APPLE)
	

## ---- MAC OS ---- ##

# possible paths for: 'j3dcore.jar'
SET(TRIAL_PATHS_DLL
  /Library/Java/JavaVirtualMachines/jdk1.7.0_45.jdk/Contents/Home/jre/lib/ext
) 

FIND_PATH(J3D_PATH j3dcore.jar ${TRIAL_PATHS_DLL})

## ---------------- ##


ELSE (APPLE)
	

## ---- LINUX ---- ##	

# possible paths for: 'libj3dcore-ogl.so'
SET(TRIAL_PATHS_DLL
  /usr/lib/jvm/java-7-openjdk-amd64/jre/bin
  /usr/lib/jvm/java-6-sun/jre/bin

  /usr/lib/jvm/java-7-openjdk-i386/jre/bin
)  

FIND_PATH(J3D_PATH libj3dcore-ogl.so ${TRIAL_PATHS_DLL})

## --------------- ##


ENDIF (APPLE)

ELSE (UNIX)


## ---- WINDOWS ---- ##

# possible paths for: 'j3dcore-ogl.dll'
SET(TRIAL_PATHS_DLL
  C:/Program\ Files/Java/jdk1.7.0_51/jre/bin
  C:/Program\ Files/Java/jdk1.8.0/jre/bin
) 

FIND_PATH(J3D_PATH j3dcore-ogl.dll ${TRIAL_PATHS_DLL})

## ----------------- ##


ENDIF (UNIX)


# flag put to 1 if all required files are found
IF(J3D_PATH)
  SET(J3D_FOUND 1)
ELSE(J3D_PATH)
  SET(J3D_FOUND 0)
ENDIF(J3D_PATH)

# outputs
MARK_AS_ADVANCED(
  J3D_FOUND
  J3D_PATH 
)
