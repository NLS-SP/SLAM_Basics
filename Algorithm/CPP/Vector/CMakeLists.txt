cmake_minimum_required(VERSION 2.8)
project(vector_study)

set(CMAKE_MACOSX_RPATH 1)

include_directories(include)

set(SOURCE_FILE src)
add_executable(vector_test vector_test.cpp
        ${SOURCE_FILE})

add_library(vectorLIB SHARED src/Vector.cpp)
