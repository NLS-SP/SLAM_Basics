cmake_minimum_required(VERSION 2.8)
project(myRotation)

set(CMAKE_CXX_STANDARD 11)

include_directories(/usr/local/include/eigen3)

find_package(Sophus REQUIRED)
include_directories(/usr/local/include)

add_executable(Rotation_test Rotation_test.cpp)

add_executable(Rotation_sophus Rotation_sophus.cpp)

target_link_libraries(Rotation_sophus ${Sophus_LIBRARIES})
target_link_libraries(Rotation_test ${Sophus_LIBRARIES})

add_executable(Rotation_Example Rotation_example.cpp)

add_executable(Eigen_test Eigen_test.cpp)
