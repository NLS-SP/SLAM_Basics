cmake_minimum_required(VERSION 3.13)
project(myICP)

include_directories(include)
include_directories(/usr/local/include/eigen3)

include_directories(/usr/local/Cellar/boost/1.69.0/include)
link_directories(/usr/local/Cellar/boost/1.69.0/lib)

set(CMAKE_CXX_STANDARD 11)

add_executable(ICP_test src/ICP_test.cpp)
