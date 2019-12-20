cmake_minimum_required(VERSION 2.8)
project(GUI_TOOL)

set(GLEW_H /usr/local/Cellar/glew/2.1.0/include)
set(GLFW_H /usr/local/Cellar/glfw/3.2.1/include)
set(GLAD_H /usr/local/include)
include_directories(${GLEW_H} ${GLFW_H} ${GLAD_H})

set(GLEW_LINK /usr/local/Cellar/glew/2.1.0/lib/libGLEW.2.1.dylib)
set(GLFW_LINK /usr/local/Cellar/glfw/3.2.1/lib/libglfw.3.dylib)
link_libraries(${OpenGL} ${GLEW_LINK} ${GLFW_LINK})

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES "src/glad.c")

set(Third_PARTY_LIBS "-framework OpenGL" "-framework GLUT")

add_executable(OpenGL_test src/OpenGL_test.cpp ${SOURCE_FILES})
target_link_libraries(OpenGL_test ${Third_PARTY_LIBS})