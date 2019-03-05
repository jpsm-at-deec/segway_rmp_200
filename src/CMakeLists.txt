# edit the following line to add all the source code files of the library
SET(sources segway_rmp200.cpp segway_rmp200_exceptions.cpp)
# edit the following line to add all the header files of the library
SET(headers segway_rmp200.h segway_rmp200_exceptions.h)

# edit the following line to find the necessary packages
FIND_PACKAGE(iriutils REQUIRED)
FIND_PACKAGE(comm REQUIRED)

# edit the following line to add the necessary include directories
INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${comm_INCLUDE_DIR})

ADD_LIBRARY(segway_rmp_200 SHARED ${sources})

#edit the following line to add the necessary system libraries (if any)
TARGET_LINK_LIBRARIES(segway_rmp_200 ${iriutils_LIBRARY})
TARGET_LINK_LIBRARIES(segway_rmp_200 ${comm_LIBRARY})

INSTALL(TARGETS segway_rmp_200
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/iri/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/iri/${PROJECT_NAME}
)

INSTALL(FILES ${headers} DESTINATION include/iri/${PROJECT_NAME})

INSTALL(FILES ../Findsegway_rmp_200.cmake DESTINATION ${CMAKE_ROOT}/Modules/)
	
ADD_SUBDIRECTORY(examples)
