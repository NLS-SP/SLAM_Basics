cmake_minimum_required(VERSION 3.16)
project(LIV_OpenGL)

set(CMAKE_CXX_STANDARD 14)

set(GLEW_H /usr/local/Cellar/glew/2.1.0_1/include)
set(GLFW_H /usr/local/Cellar/glfw/3.3.2/include)
include_directories(${GLEW_H} ${GLFW_H})

set(GLEW_LINK /usr/local/Cellar/glew/2.1.0_1/lib/libGLEW.2.1.dylib)
set(GLFW_LINK /usr/local/Cellar/glfw/3.3.2/lib/libglfw.3.dylib)
link_libraries(${OPENGL} ${GLEW_LINK} ${GLFW_LINK})

set(THIRD_PARTY "-framework OpenGL -framework GLUT")

set(Boost_NO_BOOST_CMAKE 1)
find_package(Boost REQUIRED COMPONENTS filesystem system)
set(Boost_USE_MULTITHREADED TRUE)


# CMAKE Properties
#set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -o3 -DNDBUG ${CMAKE_CXX_FLAGS}")

message(STATUS "Using CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")


set(OPENGL_VERSION 330 CACHE STRING "Available OpenGL Version")

add_definitions(-D__GL_VERSION=${OPENGL_VERSION})
message(STATUS "Using OpenGL version ${OPENGL_VERSION}.")

include_directories(include)
file(GLOB_RECURSE CPP_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${THIRD_PARTY} ${Boost_LIBRARIES})

find_package(GTest)


# 找到Eigen
include_directories("/usr/local/include/eigen3")

if(GTEST_FOUND)
    message(STATUS "Building test")
    enable_testing()
    add_subdirectory(test)
endif()

