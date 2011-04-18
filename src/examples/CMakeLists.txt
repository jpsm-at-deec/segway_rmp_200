# edit the following line to add the source code for the example and the name of the executable
ADD_EXECUTABLE(test_segway_rmp_200 test_segwayRMP200.cpp)
ADD_EXECUTABLE(motionModelData motionModelData.cpp)

# edit the following line to add the necessary libraries
TARGET_LINK_LIBRARIES(test_segway_rmp_200 segway_rmp_200 ${comm_LIBRARY}) 
TARGET_LINK_LIBRARIES(motionModelData segway_rmp_200 ${comm_LIBRARY}) 


