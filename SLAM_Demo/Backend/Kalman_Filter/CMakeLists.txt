cmake_minimum_required(VERSION 3.13)
project(Kalman_Filter)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
include_directories("/usr/local/include/eigen3")

add_executable(Kalman_Filter main.cpp)