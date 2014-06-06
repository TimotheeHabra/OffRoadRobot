#
# author: Nicolas Van der Noot
# March 3 2014
#
# This file finds the include folders (header files) and the libraries
# related to Simbody
#
# SIMBODY_FOUND :        1 if all required files found (0 otherwise)
# SIMBODY_INCLUDE_PATH : include paths (for the header files) -> for compilation
# SIMBODY_LIBRARIES :    libraries -> for linkage
#


IF (UNIX)

IF (APPLE)
	

## ---- MAC OS ---- ##



## ---------------- ##


ELSE (APPLE)
	

## ---- LINUX ---- ##	

# possible paths for: 'Simbody.h'
SET(TRIAL_PATHS_INC
  /usr/local/simbody/include
) 

# possible paths for: 'libSimTKcommon.so'
SET(TRIAL_PATHS_LIB_COM
  /usr/local/simbody/lib
) 

# possible paths for: 'libSimTKmath.so'
SET(TRIAL_PATHS_LIB_MATH
  /usr/local/simbody/lib
) 

# possible paths for: 'libSimTKsimbody.so'
SET(TRIAL_PATHS_LIB_SIMB
  /usr/local/simbody/lib
) 

FIND_PATH(SIMBODY_INCLUDE_PATH Simbody.h ${TRIAL_PATHS_INC})

FIND_LIBRARY(SIMBODY_LIBRARIES_COM libSimTKcommon.so ${TRIAL_PATHS_LIB_COM})
FIND_LIBRARY(SIMBODY_LIBRARIES_MATH libSimTKmath.so ${TRIAL_PATHS_LIB_MATH})
FIND_LIBRARY(SIMBODY_LIBRARIES_SIMB libSimTKsimbody.so ${TRIAL_PATHS_LIB_SIMB})

SET(SIMBODY_LIBRARIES ${SIMBODY_LIBRARIES_COM} ${SIMBODY_LIBRARIES_MATH} ${SIMBODY_LIBRARIES_SIMB})

## --------------- ##


ENDIF (APPLE)

ELSE (UNIX)


## ---- WINDOWS ---- ##

# possible paths for: 'Simbody.h'
SET(TRIAL_PATHS_INC
  C:/Program\ Files/Simbody/include
) 

# possible paths for: 'SimTKcommon_d.so'
SET(TRIAL_PATHS_LIB_COM
  C:/Program\ Files/Simbody/lib
) 

# possible paths for: 'SimTKmath_d.so'
SET(TRIAL_PATHS_LIB_MATH
  C:/Program\ Files/Simbody/lib
) 

# possible paths for: 'SimTKsimbody_d.so'
SET(TRIAL_PATHS_LIB_SIMB
  C:/Program\ Files/Simbody/lib
) 

FIND_PATH(SIMBODY_INCLUDE_PATH Simbody.h ${TRIAL_PATHS_INC})

FIND_LIBRARY(SIMBODY_LIBRARIES_COM SimTKcommon.lib ${TRIAL_PATHS_LIB_COM})
FIND_LIBRARY(SIMBODY_LIBRARIES_MATH SimTKmath.lib ${TRIAL_PATHS_LIB_MATH})
FIND_LIBRARY(SIMBODY_LIBRARIES_SIMB SimTKsimbody.lib ${TRIAL_PATHS_LIB_SIMB})
#FIND_LIBRARY(SIMBODY_LIBRARIES_COM SimTKcommon.lib ${TRIAL_PATHS_LIB_COM})
#FIND_LIBRARY(SIMBODY_LIBRARIES_MATH SimTKmath.lib ${TRIAL_PATHS_LIB_MATH})
#FIND_LIBRARY(SIMBODY_LIBRARIES_SIMB SimTKsimbody.lib ${TRIAL_PATHS_LIB_SIMB})

SET(SIMBODY_LIBRARIES ${SIMBODY_LIBRARIES_COM} ${SIMBODY_LIBRARIES_MATH} ${SIMBODY_LIBRARIES_SIMB})

## ----------------- ##


ENDIF (UNIX)


# flag put to 1 if all required files are found
IF(SIMBODY_INCLUDE_PATH AND SIMBODY_LIBRARIES)
  SET(SIMBODY_FOUND 1)
ELSE(SIMBODY_INCLUDE_PATH AND SIMBODY_LIBRARIES)
  SET(SIMBODY_FOUND 0)
ENDIF(SIMBODY_INCLUDE_PATH AND SIMBODY_LIBRARIES)

# outputs
MARK_AS_ADVANCED(
  SIMBODY_FOUND
  SIMBODY_INCLUDE_PATH
  SIMBODY_LIBRARIES 
)
