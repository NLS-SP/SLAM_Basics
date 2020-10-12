cmake_minimum_required(VERSION 2.8)
project(RANSAC_FIT_TO_PLANE)

include_directories(/usr/local/include/eigen3)
include_directories(include)

set(PCL_DIR /usr/local/pcl/share/pcl-1.11)
find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("/usr/local/include/eigen3")


add_executable(PLANE_FITTING src/Ransac.cpp src/PlaneModel.cpp src/data_generator.cpp main.cpp)
target_link_libraries(PLANE_FITTING ${PCL_LIBRARIES})
