cmake_minimum_required(VERSION 2.8)
project(Implicit_Surface_Reconstruction)

include_directories("/usr/local/include/eigen3")

include_directories(include)
find_package(PCL 1.11 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS thread system REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

#add_executable(implicit_reconstruction src/main.cpp)
add_executable(POSSION_Reconstruction src/main.cpp)
target_link_libraries(POSSION_Reconstruction ${PCL_LIBRARIES} ${Boost_LIBRARIES})