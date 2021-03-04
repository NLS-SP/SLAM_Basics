project(LK_Example)

add_compile_options(-std=c++17)

# OpenCV SET
#set(OpenCV_DIR /usr/local/opencv3.4.13/share/OpenCV)
#find_package(OpenCV 3.4 REQUIRED)
#include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv3.4.13/include)
#link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv3.4.13/lib)

# Eigen Set
include_directories("/usr/local/include/eigen3")
include_directories(include)
add_executable(LK_Example src/optic_flow.cpp)
target_link_libraries(LK_Example ${OpenCV_LIBS})

add_executable(direct_matching src/direct_matching.cpp)
target_link_libraries(direct_matching ${OpenCV_LIBS})