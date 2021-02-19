cmake_minimum_required(VERSION 3.5)
project(SLAM_FRONTED)

# OpenCV 2
# set(OpenCV_DIR /usr/local/opencv2.4.13/share/OpenCV)
# find_package(OpenCV 2.4 REQUIRED)
# include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv2.4.13/include)
# link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv2.4.13/lib)

# OpenCV 3
set(OpenCV_DIR /usr/local/opencv3.4.1/share/OpenCV)
find_package(OpenCV 3.4.1 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv3.4.1/include)
link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv3.4.1/lib)

# C++11 Standard
set(CMAKE_CXX_STANDARD 11)

# Eigen
# include_directories("/usr/local/Cellar/eigen/eigen3")
# FIND_PACKAGE(eigen3 REQUIRED)
include_directories("/usr/local/include/eigen3")

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

set(THIRD_PARTY_LIBS ${OpenCV_LIBS} ${Sophus_LIBRARIES} g2o_core g2o_stuff g2o_types_sba)

add_subdirectory(OpenCV_ORB)
add_subdirectory(Features)
add_subdirectory(Pose_Estimation)
add_subdirectory(Map)
add_subdirectory(Viewer)
add_subdirectory(LASER_Projection)
add_subdirectory(LK_Example)
