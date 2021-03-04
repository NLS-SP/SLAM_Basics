project(Stero_Depth_Estimate)

add_compile_options(-std=c++11)

# Eigen Set
include_directories("/usr/local/include/eigen3")
include_directories(include)
add_executable(Stereo_Vision src/stereo_vision.cpp)
target_link_libraries(Stereo_Vision ${OpenCV_LIBS})