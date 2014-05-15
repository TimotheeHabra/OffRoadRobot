#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# This file finds the include folders (header files) and the libraries
# related to Libxml2
#
# LIBXML2_FOUND :        1 if all required files found (0 otherwise)
# LIBXML2_INCLUDE_PATH : include paths (for the header files) -> for compilation
# LIBXML2_LIBRARIES :    libraries -> for linkage
#


IF (UNIX)

IF (APPLE)
	

## ---- MAC OS ---- ##

# possible paths for: 'iconv.h'
SET(TRIAL_PATHS_INC_ICONV
  /usr/include
) 

# possible paths for: 'xpath.h'
SET(TRIAL_PATHS_INC_LIBXML
  /usr/include/libxml2/libxml
) 

# possible paths for: 'libxml/xmlmemory.h'
SET(TRIAL_PATHS_INC_LIBXML2
  /usr/include/libxml2
) 

# possible paths for: 'libxml2.dylib'
SET(TRIAL_PATHS_LIB_LIBXML2
  /usr/lib
) 

FIND_PATH(LIBXML2_INCLUDE_PATH_ICONV iconv.h ${TRIAL_PATHS_INC_ICONV})
FIND_PATH(LIBXML2_INCLUDE_PATH_LIBXML xpath.h ${TRIAL_PATHS_INC_LIBXML})
FIND_PATH(LIBXML2_INCLUDE_PATH_XMLMEMORY libxml/xmlmemory.h ${TRIAL_PATHS_INC_LIBXML2})

FIND_LIBRARY(LIBXML2_LIBRARIES_LIBXML2 libxml2.dylib ${TRIAL_PATHS_LIB_LIBXML2})

SET(LIBXML2_INCLUDE_PATH ${LIBXML2_INCLUDE_PATH_ICONV} ${LIBXML2_INCLUDE_PATH_LIBXML} ${LIBXML2_INCLUDE_PATH_XMLMEMORY})
SET(LIBXML2_LIBRARIES ${LIBXML2_LIBRARIES_LIBXML2})

## ---------------- ##


ELSE (APPLE)
	

## ---- LINUX ---- ##	

# possible paths for: 'iconv.h'
SET(TRIAL_PATHS_INC_ICONV
  /usr/include
) 

# possible paths for: 'xpath.h'
SET(TRIAL_PATHS_INC_LIBXML
  /usr/include/libxml2/libxml
) 

# possible paths for: 'libxml/xmlmemory.h'
SET(TRIAL_PATHS_INC_LIBXML2
  /usr/include/libxml2
) 

# possible paths for: 'libxml2.so'
SET(TRIAL_PATHS_LIB_LIBXML2
  /usr/lib/i386-linux-gnu
) 

FIND_PATH(LIBXML2_INCLUDE_PATH_ICONV iconv.h ${TRIAL_PATHS_INC_ICONV})
FIND_PATH(LIBXML2_INCLUDE_PATH_LIBXML xpath.h ${TRIAL_PATHS_INC_LIBXML})
FIND_PATH(LIBXML2_INCLUDE_PATH_XMLMEMORY libxml/xmlmemory.h ${TRIAL_PATHS_INC_LIBXML2})

FIND_LIBRARY(LIBXML2_LIBRARIES_LIBXML2 libxml2.so ${TRIAL_PATHS_LIB_LIBXML2})

SET(LIBXML2_INCLUDE_PATH ${LIBXML2_INCLUDE_PATH_ICONV} ${LIBXML2_INCLUDE_PATH_LIBXML} ${LIBXML2_INCLUDE_PATH_XMLMEMORY})
SET(LIBXML2_LIBRARIES ${LIBXML2_LIBRARIES_LIBXML2})

## --------------- ##


ENDIF (APPLE)

ELSE (UNIX)


## ---- WINDOWS ---- ##

# possible paths for: 'iconv.h'
SET(TRIAL_PATHS_INC_ICONV
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/include
) 

# possible paths for: 'xpath.h'
SET(TRIAL_PATHS_INC_LIBXML
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/include/libxml
) 

# possible paths for: 'iconv.lib'
SET(TRIAL_PATHS_LIB_ICONV
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/lib
) 

# possible paths for: 'libxml2.lib'
SET(TRIAL_PATHS_LIB_LIBXML2
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/lib
) 

FIND_PATH(LIBXML2_INCLUDE_PATH_ICONV iconv.h ${TRIAL_PATHS_INC_ICONV})
FIND_PATH(LIBXML2_INCLUDE_PATH_LIBXML xpath.h ${TRIAL_PATHS_INC_LIBXML})

FIND_LIBRARY(LIBXML2_LIBRARIES_ICONV iconv.lib ${TRIAL_PATHS_LIB_ICONV})
FIND_LIBRARY(LIBXML2_LIBRARIES_LIBXML2 libxml2.lib ${TRIAL_PATHS_LIB_LIBXML2})

SET(LIBXML2_INCLUDE_PATH ${LIBXML2_INCLUDE_PATH_ICONV} ${LIBXML2_INCLUDE_PATH_LIBXML})
SET(LIBXML2_LIBRARIES ${LIBXML2_LIBRARIES_ICONV} ${LIBXML2_LIBRARIES_LIBXML2})

## ----------------- ##


ENDIF (UNIX)


# flag put to 1 if all required files are found
IF(LIBXML2_INCLUDE_PATH AND LIBXML2_LIBRARIES)
  SET(LIBXML2_FOUND 1)
ELSE(LIBXML2_INCLUDE_PATH AND LIBXML2_LIBRARIES)
  SET(LIBXML2_FOUND 0)
ENDIF(LIBXML2_INCLUDE_PATH AND LIBXML2_LIBRARIES)

# outputs
MARK_AS_ADVANCED(
  LIBXML2_FOUND
  LIBXML2_INCLUDE_PATH
  LIBXML2_LIBRARIES 
)
