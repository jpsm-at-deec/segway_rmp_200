SET(name segway_rmp_200)
#edit the following line to add the librarie's header files
SET(header_files segway_rmp200.h segway_rmp200_exceptions.h)

FIND_PATH(${name}_INCLUDE_DIRS ${header_files} /usr/include/iri/${name} /usr/local/include/iri/${name})

FIND_LIBRARY(${name}_LIBRARIES
    NAMES ${name}
    PATHS /usr/lib /usr/lib/iri/${name} /usr/local/lib /usr/local/lib/iri/${name})

SET(${name}_INCLUDE_DIR ${${name}_INCLUDE_DIRS})
SET(${name}_LIBRARY     ${${name}_LIBRARIES})

IF (${name}_INCLUDE_DIRS AND ${name}_LIBRARIES)
   SET(${name}_FOUND TRUE)
ENDIF (${name}_INCLUDE_DIRS AND ${name}_LIBRARIES)

IF (${name}_FOUND)
   IF (NOT ${name}_FIND_QUIETLY)
      MESSAGE(STATUS "Found ${name}: ${${name}_LIBRARIES}")
   ENDIF (NOT ${name}_FIND_QUIETLY)
ELSE (${name}_FOUND)
   IF (${name}_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find ${name}")
   ENDIF (${name}_FIND_REQUIRED)
ENDIF (${name}_FOUND)        