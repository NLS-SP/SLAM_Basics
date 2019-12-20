project(Features)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE DEBUG)

add_executable(ORB_Compute ORB_Compute.cpp)
target_link_libraries(ORB_Compute ${OpenCV_LIBS})

add_executable(SIFT_Compute SIFT_Compute.cpp)
target_link_libraries(SIFT_Compute ${OpenCV_LIBS})