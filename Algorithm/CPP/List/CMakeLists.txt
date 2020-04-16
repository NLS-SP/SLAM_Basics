cmake_minimum_required(VERSION 2.8)
project(List_study)

set(CMAKE_MACOSX_RPATH 1)

include_directories(../List/include)

add_executable(List_test ../List/List_test.cpp)

