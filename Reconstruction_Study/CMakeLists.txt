cmake_minimum_required(VERSION 2.8)
project(Reconstruction_Study)

set(CMAKE_CXX_STANDARD 14)
include_directories("/usr/local/include/eigen3")

add_subdirectory(3L_IBS)
add_subdirectory(PolyFitting)
add_subdirectory(TSDF_Reconstruct)
add_subdirectory(Octree)