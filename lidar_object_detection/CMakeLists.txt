cmake_minimum_required(VERSION 2.8)
project(lidar_object_detection)

include_directories("/usr/local/include/eigen3")

include_directories(include)
find_package(PCL 1.11 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS thread system REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

#add_executable(implicit_reconstruction src/main.cpp )
add_executable(lidar_object_detect src/main.cpp src/processPointCloud.cpp include/render/render.cpp)
target_link_libraries(lidar_object_detect ${PCL_LIBRARIES} ${Boost_LIBRARIES})