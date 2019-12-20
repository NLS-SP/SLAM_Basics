project(Window_Viewer)

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_STANDARD 11)

find_package(Pangolin 0.4 REQUIRED)
find_package(Threads REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(HelloPangolin helloPangolin.cpp)
target_link_libraries(HelloPangolin ${Pangolin_LIBRARIES})

add_executable(HelloPangolinOffScreen helloPangolinOffScreen.cpp)
target_link_libraries(HelloPangolinOffScreen ${Pangolin_LIBRARIES})

add_executable(HelloPangolinThreads helloPangolinThreads.cpp)
target_link_libraries(HelloPangolinThreads ${Pangolin_LIBRARIES})

