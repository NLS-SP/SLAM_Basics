cmake_minimum_required(VERSION 2.8)
project(VIO_Study)

# OpenCV Setting.
set(OpenCV_DIR /usr/local/opencv3.4.10/share/OpenCV)
find_package(OpenCV 3.4 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv3.4.10/include)
link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv3.4.10/lib)
include_directories(${OpenCV_INCLUDE_DIRS})

# C++14 Standard.
set(CMAKE_CXX_STANDARD 14)

# Eigen.
include_directories("/usr/local/include/eigen3")

set(THIRD_PARTY_LIBS ${OpenCV_LIBS})

add_subdirectory(Introduction)