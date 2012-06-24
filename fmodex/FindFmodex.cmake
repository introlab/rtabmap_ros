# - Find Fmodex
# This module finds an installed Fmod package.
#
# It sets the following variables:
#  Fmodex_FOUND       - Set to false, or undefined, if Fmod isn't found.
#  Fmodex_INCLUDE_DIRS - The Fmod include directory.
#  Fmodex_LIBRARIES    - The Fmod library to link against.
#
# 

SET(Fmodex_ROOT)

# Add ROS Fmodex directory if ROS is installed
FIND_PROGRAM(ROSPACK_EXEC NAME rospack PATHS)  
IF(ROSPACK_EXEC)  
	EXECUTE_PROCESS(COMMAND ${ROSPACK_EXEC} find fmodex 
			   	    OUTPUT_VARIABLE Fmodex_ROS_PATH
					OUTPUT_STRIP_TRAILING_WHITESPACE
					WORKING_DIRECTORY "./"
	)
	IF(Fmodex_ROS_PATH)
	    MESSAGE(STATUS "Found Fmodex ROS pkg : ${Fmodex_ROS_PATH}")
	    SET(Fmodex_ROOT
	        ${Fmodex_ROS_PATH}/fmodex
	        ${Fmodex_ROOT}
	    )
	ENDIF(Fmodex_ROS_PATH)
ENDIF(ROSPACK_EXEC)
     
FIND_PATH(Fmodex_INCLUDE_DIRS fmod.h PATHS ${Fmodex_ROOT}/include)

IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
   FIND_LIBRARY(Fmodex_LIBRARIES NAMES fmodex64 PATHS ${Fmodex_ROOT}/lib)
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)
IF(NOT Fmodex_LIBRARIES)
   FIND_LIBRARY(Fmodex_LIBRARIES NAMES fmodex PATHS ${Fmodex_ROOT}/lib)
ENDIF(NOT Fmodex_LIBRARIES)

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
      MESSAGE(FATAL_ERROR "Could not find Fmodex... (aka libfmodex)")
   ENDIF (Fmodex_FIND_REQUIRED)
ENDIF (Fmodex_FOUND)
