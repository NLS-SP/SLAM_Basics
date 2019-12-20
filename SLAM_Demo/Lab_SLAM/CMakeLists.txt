cmake_minimum_required(VERSION 2.8)
project(Lab_SLAM)

# OpenCV3
set(OpenCV_DIR /usr/local/opencv3.4.5/share/OpenCV)
find_package(OpenCV 3.4 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv3.4.5/include)
link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv3.4.5/lib)

# C++11 Standard
set(CMAKE_CXX_STANDARD 11)

# EIGEN
include_directories("/usr/local/include/eigen3")

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Set third parities
SET(THIRD_PARTY_LIBS ${OpenCV_LIBS} ${Sophus_LIBRARIES})

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED
        src/Camera.cpp
        src/TestCase.cpp)