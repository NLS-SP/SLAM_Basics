cmake_minimum_required(VERSION 3.13)
project(GaussNewton_CPP)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
include_directories("/usr/local/include/eigen3")

set(OpenCV_DIR /usr/local/opencv3.4.1/share/OpenCV)
find_package(OpenCV 3.4.1 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv3.4.1/include)
link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv3.4.1/lib)


add_executable(GaussNewton src/GaussNewton.cpp)
target_link_libraries(GaussNewton ${OpenCV_LIBS})