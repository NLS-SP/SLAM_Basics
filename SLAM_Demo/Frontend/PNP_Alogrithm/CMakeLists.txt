project(PNP)

set(CMAKE_CXX_STANDARD 14)

set(OpenCV_DIR /usr/local/opencv3.4.1/share/OpenCV)
find_package(OpenCV 3.4.1 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv3.4.1/include)
link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv3.4.1/lib)

include_directories("/usr/local/include/eigen3")

find_package(G2O REQUIRED)
include_directories(/usr/local/include)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(P3P_Solver P3P_Solve.cpp)
target_link_libraries(P3PSolve)

add_executable(PNP_BA PNP_BA.cpp)
target_link_libraries(PNP_BA ${OpenCV_LIBS} /usr/local/lib/libg2o_core.dylib)