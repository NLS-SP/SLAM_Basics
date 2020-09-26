cmake_minimum_required(VERSION 2.8)
project(RANSAC_FIT_TO_PLANE)

include_directories(/usr/local/include/eigen3)
include_directories(include)

add_executable(PLANE_FITTING src/Ransac.cpp main.cpp)
