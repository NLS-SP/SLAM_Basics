cmake_minimum_required(VERSION 2.8)
project(Implicit_Surface_Reconstruction)

include_directories("/usr/local/include/eigen3")

include_directories(include)

add_executable(implicit_reconstruction src/main.cpp)