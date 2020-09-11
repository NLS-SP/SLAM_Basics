cmake_minimum_required(VERSION 2.8)
project(TSDF_Reconstruct)

include_directories("/usr/local/include/eigen3")

include_directories(include)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Octree_Representation src/main src/Octree.cc)
target_link_libraries(Octree_Representation ${PCL_LIBRARIES})

