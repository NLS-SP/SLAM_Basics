project(ORB_Main)
set(CMAKE_CXX_STANDARD 11)

add_executable(ORB_main ORB_main.cpp)
target_link_libraries(ORB_main ${OpenCV_LIBS})