cmake_minimum_required(VERSION 2.8)
project(GUI_TOOL)

include_directories(include)

set(GLEW_H /usr/local/Cellar/glew/2.1.0/include)
set(GLFW_H /usr/local/Cellar/glfw/3.3/include)
set(GLAD_H /usr/local/include)
include_directories(${GLEW_H} ${GLFW_H} ${GLAD_H})

set(GLEW_LINK /usr/local/Cellar/glew/2.1.0/lib/libGLEW.2.1.dylib)
set(GLFW_LINK /usr/local/Cellar/glfw/3.3/lib/libglfw.3.dylib)
link_libraries(${OpenGL} ${GLEW_LINK} ${GLFW_LINK})

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES "src/glad.c")

set(Third_PARTY_LIBS "-framework OpenGL" "-framework GLUT")

add_executable(OpenGL_test src/OpenGL_test.cpp ${SOURCE_FILES})
target_link_libraries(OpenGL_test ${Third_PARTY_LIBS})

add_executable(helloWindow src/helloWindow.cpp ${SOURCE_FILES})
target_link_libraries(helloWindow ${Third_PARTY_LIBS})

add_executable(helloTriangle src/helloTriangle.cpp ${SOURCE_FILES})
target_link_libraries(helloTriangle ${Third_PARTY_LIBS})

add_executable(helloRectangle src/helloRectangle.cpp ${SOURCE_FILES})
target_link_libraries(helloRectangle ${Third_PARTY_LIBS})

add_executable(hello_different_triangles src/hello_different_triangles.cpp ${SOURCE_FILES})
target_link_libraries(hello_different_triangles ${Third_PARTY_LIBS})

add_executable(hello_shader_triangle src/hello_shader_triangle.cpp ${SOURCE_FILES})
target_link_libraries(hello_shader_triangle ${Third_PARTY_LIBS})

add_executable(hello_shader_Programing src/hello_shader_Programing.cpp ${SOURCE_FILES})
target_link_libraries(hello_shader_Programing ${Third_PARTY_LIBS})

add_executable(hello_triangle_change src/hello_triangle_change.cpp ${SOURCE_FILES})
target_link_libraries(hello_triangle_change ${Third_PARTY_LIBS})

add_executable(hello_glsl_triangle src/hello_glsl_triangle.cpp ${SOURCE_FILES})
target_link_libraries(hello_glsl_triangle ${Third_PARTY_LIBS})

add_executable(hello_shader_file src/hello_shader_file.cpp ${SOURCE_FILES})
target_link_libraries(hello_shader_file ${Third_PARTY_LIBS})