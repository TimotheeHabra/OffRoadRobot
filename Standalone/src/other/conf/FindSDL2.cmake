#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# This file finds the include folders (header files) and the libraries
# related to SDL2
#
# SDL2_FOUND :        1 if all required files found (0 otherwise)
# SDL2_INCLUDE_PATH : include paths (for the header files) -> for compilation
# SDL2_LIBRARIES :    libraries -> for linkage
#


IF (UNIX)

IF (APPLE)
	

## ---- MAC OS ---- ##

# possible paths for: 'SDL_assert.h'
SET(TRIAL_PATHS_INC_SDL_ASSERT
  /usr/local/include/SDL2
)

# possible paths for: 'SDL_active.h'
SET(TRIAL_PATHS_INC_SDL_ACT
  /Library/Frameworks/SDL.framework/Versions/A/Headers/
)

# possible paths for: 'libSDL2.dylib'
SET(TRIAL_PATHS_LIB_SDL2
  /usr/local/lib
)

FIND_PATH(SDL2_INC_SDL_ASSERT SDL_assert.h ${TRIAL_PATHS_INC_SDL_ASSERT})
FIND_PATH(SDL2_INC_SDL_ACT SDL_active.h ${TRIAL_PATHS_INC_SDL_ACT})

FIND_LIBRARY(SDL2_LIBRARIES_SDL2 libSDL2.dylib ${TRIAL_PATHS_LIB_SDL2})

SET(SDL2_LIBRARIES ${SDL2_LIBRARIES_SDL2} "-framework Cocoa")
SET(SDL2_INCLUDE_PATH ${SDL2_INC_SDL_ASSERT} ${SDL2_INC_SDL_ACT})

## ---------------- ##


ELSE (APPLE)
	

## ---- LINUX ---- ##	

# possible paths for: 'SDL.h'
SET(TRIAL_PATHS_INC
  /usr/local/include/SDL2
)

# possible paths for: 'libSDL2.so'
SET(TRIAL_PATHS_LIB_SDL2
  /usr/local/lib
)

FIND_PATH(SDL2_INCLUDE_PATH SDL.h ${TRIAL_PATHS_INC})

FIND_LIBRARY(SDL2_LIBRARIES_SDL2 libSDL2.so ${TRIAL_PATHS_LIB_SDL2})

SET(SDL2_LIBRARIES ${SDL2_LIBRARIES_SDL2})

## --------------- ##


ENDIF (APPLE)

ELSE (UNIX)


## ---- WINDOWS ---- ##

# possible paths for: 'SDL.h'
SET(TRIAL_PATHS_INC
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/include/SDL
)

# possible paths for: 'SDL2.lib'
SET(TRIAL_PATHS_LIB_SDL2
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/lib
)

# possible paths for: 'SDL2main.lib'
SET(TRIAL_PATHS_LIB_MAIN
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/lib
)

# possible paths for: 'SDL2test.lib'
SET(TRIAL_PATHS_LIB_TEST
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/lib
)

FIND_PATH(SDL2_INCLUDE_PATH SDL.h ${TRIAL_PATHS_INC})

FIND_LIBRARY(SDL2_LIBRARIES_SDL2 SDL2.lib ${TRIAL_PATHS_LIB_SDL2})
FIND_LIBRARY(SDL2_LIBRARIES_MAIN SDL2main.lib ${TRIAL_PATHS_LIB_MAIN})
FIND_LIBRARY(SDL2_LIBRARIES_TEST SDL2test.lib ${TRIAL_PATHS_LIB_TEST})

SET(SDL2_LIBRARIES ${SDL2_LIBRARIES_SDL2} ${SDL2_LIBRARIES_MAIN} ${SDL2_LIBRARIES_TEST})

## ----------------- ##


ENDIF (UNIX)


# flag put to 1 if all required files are found
IF(SDL2_INCLUDE_PATH AND SDL2_LIBRARIES)
  SET(SDL2_FOUND 1)
ELSE(SDL2_INCLUDE_PATH AND SDL2_LIBRARIES)
  SET(SDL2_FOUND 0)
ENDIF(SDL2_INCLUDE_PATH AND SDL2_LIBRARIES)

# outputs
MARK_AS_ADVANCED(
  SDL2_FOUND
  SDL2_INCLUDE_PATH
  SDL2_LIBRARIES 
)
