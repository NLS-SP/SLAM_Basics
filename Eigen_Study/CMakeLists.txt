cmake_minimum_required(VERSION 2.8)
project(Eigen_Project)

include_directories("/usr/local/include/eigen3")

add_executable(Eigen_Example src/Eigen_Tutorial.cpp)