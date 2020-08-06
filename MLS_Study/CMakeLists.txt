cmake_minimum_required(VERSION 3.5)
project(MLS_Study)

set(PCL_DIR /usr/local/pcl/share/pcl-1.11)
find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS thread system REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_executable(mls_example src/mls_study.cpp)
target_link_libraries(mls_example ${PCL_LIBRARIES} ${Boost_LIBRARIES})
