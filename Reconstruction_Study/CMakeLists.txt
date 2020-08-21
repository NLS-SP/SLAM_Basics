cmake_minimum_required(VERSION 2.8)

project(TSDF_Representation)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Release" FORCE)

set(CMAKE_SKIP_BUILD_RPATH FALSE)
# 找PCL库
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(PCL_DIR /usr/local/pcl/share/pcl-1.11)
find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
# 找Boost库
find_package(Boost COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
# Create Library
add_library(TSDF_LIB SHARED src/octree.cpp
                            src/tsdf_volume_octree.cpp
                            src/tsdf_interface.cpp
                            src/marching_cubes_tsdf_octree.cpp)
target_link_libraries(TSDF_LIB ${PCL_LIBRARIES})

# Create the execubtales
add_executable(tsdf2mesh tsdf2mesh.cpp)
target_link_libraries(tsdf2mesh ${PCL_LIBRARIES} TSDF_LIB)
