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

add_executable(hello_Camera hello_Camera.cpp)
target_link_libraries(hello_Camera ${Pangolin_LIBRARIES})

add_executable(hello_GUI_Basic hello_GUI_Basic.cpp)
target_link_libraries(hello_GUI_Basic ${Pangolin_LIBRARIES})

add_executable(hello_image_Pangolin hello_image_Pangolin.cpp)
target_link_libraries(hello_image_Pangolin ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

add_executable(hello_Elastic_Fusion_GUI hello_Elastic_Fusion_GUI.cpp)
target_link_libraries(hello_Elastic_Fusion_GUI ${Pangolin_LIBRARIES} ${OpenCV_LIBS})





