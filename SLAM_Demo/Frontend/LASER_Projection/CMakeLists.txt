project(LaserImage_Projection)

add_compile_options(-std=c++17)

# OpenCV SET
#set(OpenCV_DIR /usr/local/opencv3.4.13/share/OpenCV)
#find_package(OpenCV 3.4 REQUIRED)

#include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv3.4.13/include)
#link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv3.4.13/lib)

# PCL 设置
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)
add_executable(LaserImage_Projection src/Spherical_View_Projection.cpp src/main.cpp)
target_link_libraries(LaserImage_Projection ${PCL_LIBRARIES} ${OpenCV_LIBS})