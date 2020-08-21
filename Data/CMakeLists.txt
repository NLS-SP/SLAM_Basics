cmake_minimum_required(VERSION 2.8)
project(bin_to_pcd)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(PCL_DIR /usr/local/pcl/share/pcl-1.11)
find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_executable(bin_to_pcd src/bin_to_pcd.cpp)
target_link_libraries(bin_to_pcd ${PCL_LIBRARIES} ${Boost_LIBRARIES})
