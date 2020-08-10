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


add_executable(BASIC_ICP src/ICP.cpp)

add_executable(Registration src/Registration.cpp src/main.cpp)
target_link_libraries(Registration ${PCL_LIBRARIES} ${libnabo_LIBRARIES})