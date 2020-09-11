cmake_minimum_required(VERSION 2.8)
project(RGBD_SLAM)

include_directories("/usr/local/include/eigen3")

find_package(PCL REQUIRED COMPONENTS common io)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})

set(OpenCV_DIR /usr/local/opencv3.4.10/share/OpenCV)
find_package(OpenCV 3.4 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv3.4.10/include)
link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv3.4.10/lib)

add_executable(RGBD_SLAM src/rgbd_slam.cpp)
target_link_libraries(RGBD_SLAM ${PCL_LIBRARIES} ${OpenCV_LIBS})