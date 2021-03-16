cmake_minimum_required(VERSION 3.5)
project(SLAM_BACKEND)

# C++ 14 Standard
set(CMAKE_CXX_STANDARD 14)
set(OpenCV_DIR /usr/local/opencv3.4.1/share/OpenCV)
find_package(OpenCV 3.4.1 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv3.4.1/include)
link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv3.4.1/lib)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(glog REQUIRED)
find_package(gflags REQUIRED)

add_subdirectory(CERES_Study)
add_subdirectory(GaussNewton_CPP)