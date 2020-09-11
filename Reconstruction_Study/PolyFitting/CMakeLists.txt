cmake_minimum_required(VERSION 2.8)
project(PolySurface_Fitting)

include_directories("/usr/local/include/eigen3")

include_directories(include)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PolyFititng_Representation src/main.cpp)
target_link_libraries(PolyFititng_Representation ${PCL_LIBRARIES})