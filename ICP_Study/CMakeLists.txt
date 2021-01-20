cmake_minimum_required(VERSION 3.16)
project(BASIC_ICP)
include_directories("/usr/local/include/eigen3")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(PCL_DIR /usr/local/pcl/share/pcl-1.11)
find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(libnabo REQUIRED PATHS ${LIBNABO_INSTALL_DIR})
include_directories(${libnabo_INCLUDE_DIRS})

include_directories(include)

find_package(Boost COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(BASIC_ICP src/ICP.cpp)

add_executable(Registration src/Registration.cpp src/main.cpp src/Tool.cpp)
target_link_libraries(Registration ${PCL_LIBRARIES} ${libnabo_LIBRARIES})

add_executable(PCL_NDT_Registration src/PCL_NDT_Registration.cpp)
target_link_libraries(PCL_NDT_Registration ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(PCL_ICP_Registration src/PCL_ICP_Registration.cpp)
target_link_libraries(PCL_ICP_Registration ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(PCL_NORMAL_EXTRACTION src/PCL_NORMAL_EXTRACTION.cpp)
target_link_libraries(PCL_NORMAL_EXTRACTION ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(PCL_HARRIS3D_EXTRACTION src/PCL_HARRIS3D_EXTRACTION.cpp)
target_link_libraries(PCL_HARRIS3D_EXTRACTION ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(PCL_NARF_EXTRACTION src/PCL_NARF_EXTRACTION.cpp)
target_link_libraries(PCL_NARF_EXTRACTION ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(PCL_TRIANGULATE src/PCL_TRIANGULATE.cpp)
target_link_libraries(PCL_TRIANGULATE ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(PLY_Viewer src/PLY_Viewer.cpp)
target_link_libraries(PLY_Viewer ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(PCL_BSpline_Reconstruction src/PCL_BSpline_Surface_Reconstruction.cpp)
target_link_libraries(PCL_BSpline_Reconstruction ${PCL_LIBRARIES} ${Boost_LIBRARIES})


add_executable(PCL_KDTREE src/PCL_KDTREE.cpp)
target_link_libraries(PCL_KDTREE ${PCL_LIBRARIES} ${Boost_LIBRARIES})