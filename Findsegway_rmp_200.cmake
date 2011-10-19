FIND_PATH(segway_rmp_200_INCLUDE_DIR segway_rmp200.h segway_rmp200_exceptions.h /usr/include/iridrivers /usr/local/include/iridrivers)

FIND_LIBRARY(segway_rmp_200_LIBRARY
    NAMES segway_rmp_200
    PATHS /usr/lib/iridrivers /usr/local/lib /usr/local/lib/iridrivers) 

IF (segway_rmp_200_INCLUDE_DIR AND segway_rmp_200_LIBRARY)
   SET(segway_rmp_200_FOUND TRUE)
ENDIF (segway_rmp_200_INCLUDE_DIR AND segway_rmp_200_LIBRARY)

IF (segway_rmp_200_FOUND)
   IF (NOT segway_rmp_200_FIND_QUIETLY)
      MESSAGE(STATUS "Found Segway RMP200 driver: ${segway_rmp_200_LIBRARY}")
   ENDIF (NOT segway_rmp_200_FIND_QUIETLY)
ELSE (segway_rmp_200_FOUND)
   IF (segway_rmp_200_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find segway RMP200 driver")
   ENDIF (segway_rmp_200_FIND_REQUIRED)
ENDIF (segway_rmp_200_FOUND)

