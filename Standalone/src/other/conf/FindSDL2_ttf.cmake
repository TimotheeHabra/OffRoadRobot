#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# This file finds the include folders (header files) and the libraries
# related to SDL2_ttf
#
# SDL2TTF_FOUND :        1 if all required files found (0 otherwise)
# SDL2TTF_INCLUDE_PATH : include paths (for the header files) -> for compilation
# SDL2TTF_LIBRARIES :    libraries -> for linkage
#


IF (UNIX)

IF (APPLE)
	

## ---- MAC OS ---- ##

# possible paths for: 'SDL_ttf.h'
SET(TRIAL_PATHS_INC
  /usr/local/include/SDL2
) 

# possible paths for: 'libSDL2_ttf.dylib'
SET(TRIAL_PATHS_LIB
  /usr/local/lib
) 

FIND_PATH(SDL2TTF_INCLUDE_PATH SDL_ttf.h ${TRIAL_PATHS_INC})

FIND_LIBRARY(SDL2TTF_LIBRARIES libSDL2_ttf.dylib ${TRIAL_PATHS_LIB})

## ---------------- ##


ELSE (APPLE)
	

## ---- LINUX ---- ##	

# possible paths for: 'SDL_ttf.h'
SET(TRIAL_PATHS_INC
  /usr/local/include/SDL2
) 

# possible paths for: 'libSDL2_ttf.so'
SET(TRIAL_PATHS_LIB
  /usr/local/lib
) 

FIND_PATH(SDL2TTF_INCLUDE_PATH SDL_ttf.h ${TRIAL_PATHS_INC})

FIND_LIBRARY(SDL2TTF_LIBRARIES libSDL2_ttf.so ${TRIAL_PATHS_LIB})

## --------------- ##


ENDIF (APPLE)

ELSE (UNIX)


## ---- WINDOWS ---- ##

# possible paths for: 'SDL_ttf.h'
SET(TRIAL_PATHS_INC
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/include
) 

# possible paths for: 'SDL2_ttf.lib'
SET(TRIAL_PATHS_LIB
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/lib
) 

FIND_PATH(SDL2TTF_INCLUDE_PATH SDL_ttf.h ${TRIAL_PATHS_INC})

FIND_LIBRARY(SDL2TTF_LIBRARIES SDL2_ttf.lib ${TRIAL_PATHS_LIB})

## ----------------- ##


ENDIF (UNIX)


# flag put to 1 if all required files are found
IF(SDL2TTF_INCLUDE_PATH AND SDL2TTF_LIBRARIES)
  SET(SDL2TTF_FOUND 1)
ELSE(SDL2TTF_INCLUDE_PATH AND SDL2TTF_LIBRARIES)
  SET(SDL2TTF_FOUND 0)
ENDIF(SDL2TTF_INCLUDE_PATH AND SDL2TTF_LIBRARIES)

# outputs
MARK_AS_ADVANCED(
  SDL2TTF_FOUND
  SDL2TTF_INCLUDE_PATH
  SDL2TTF_LIBRARIES 
)
