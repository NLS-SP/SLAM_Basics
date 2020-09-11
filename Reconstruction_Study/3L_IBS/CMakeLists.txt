cmake_minimum_required(VERSION 2.8)
project(3L_IBS)

include_directories("/usr/local/include/eigen3")

include_directories(include)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(3L_Representation src/3L_IBS.cpp
                                 src/main.cpp)
target_link_libraries(3L_Representation ${PCL_LIBRARIES})

add_executable(BSplineFitting src/BSplineFitting.cpp)
target_link_libraries(BSplineFitting ${PCL_LIBRARIES})