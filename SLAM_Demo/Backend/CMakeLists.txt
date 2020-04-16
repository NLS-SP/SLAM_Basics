cmake_minimum_required(VERSION 3.5)
project(SLAM_BACKEND)

# C++ 14 Standard
set(CMAKE_CXX_STANDARD 14)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(glog REQUIRED)
find_package(gflags REQUIRED)

add_subdirectory(CERES_Study)