# - Find Fmodex
# This module finds an installed Fmod package.
#
# It sets the following variables:
#  Fmodex_FOUND       - Set to false, or undefined, if Fmod isn't found.
#  Fmodex_INCLUDE_DIRS - The Fmod include directory.
#  Fmodex_LIBRARIES    - The Fmod library to link against.
#
# 
     
FIND_PATH(Fmodex_INCLUDE_DIRS fmod.h)

IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
   FIND_LIBRARY(Fmodex_LIBRARIES NAMES fmodex64)
ELSE(CMAKE_SIZEOF_VOID_P EQUAL 8)
   FIND_LIBRARY(Fmodex_LIBRARIES NAMES fmodex)
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)

IF (Fmodex_INCLUDE_DIRS AND Fmodex_LIBRARIES)
   SET(Fmodex_FOUND TRUE)
ENDIF (Fmodex_INCLUDE_DIRS AND Fmodex_LIBRARIES)

IF (Fmodex_FOUND)
   # show which Fmod was found only if not quiet
   IF (NOT Fmodex_FIND_QUIETLY)
      MESSAGE(STATUS "Found Fmod: ${Fmodex_LIBRARIES}")
   ENDIF (NOT Fmodex_FIND_QUIETLY)
ELSE (Fmodex_FOUND)
   # fatal error if Fmod is required but not found
   IF (Fmodex_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Fmod...")
   ENDIF (Fmodex_FIND_REQUIRED)
ENDIF (Fmodex_FOUND)
