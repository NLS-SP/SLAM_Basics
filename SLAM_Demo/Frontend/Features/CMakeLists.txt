project(Features)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE DEBUG)

add_executable(ORB_Compute ORB_Compute.cpp)
target_link_libraries(ORB_Compute ${OpenCV_LIBS})

add_executable(SIFT_Compute SIFT_Compute.cpp)
target_link_libraries(SIFT_Compute ${OpenCV_LIBS})

add_executable(Hough_Line Hough_Line.cpp)
target_link_libraries(Hough_Line ${OpenCV_LIBS})

add_executable(OpenCV_Edge OpenCV_Edge.cpp)
target_link_libraries(OpenCV_Edge ${OpenCV_LIBS})