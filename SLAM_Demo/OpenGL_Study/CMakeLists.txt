cmake_minimum_required(VERSION 2.8)
project(OpenGL_Study)

include_directories(include)

set(GLEW_H /usr/local/Cellar/glew/2.1.0_1/include)
set(GLFW_H /usr/local/Cellar/glfw/3.3.2/include)
set(GLAD_H /usr/local/include)
set(ASSIMP_H /usr/local/include)
include_directories(${GLEW_H} ${GLFW_H} ${GLAD_H} ${ASSIMP_H})


set(GLEW_LINK /usr/local/Cellar/glew/2.1.0_1/lib/libGLEW.2.1.dylib)
set(GLFW_LINK /usr/local/Cellar/glfw/3.3.2/lib/libglfw.3.dylib)
set(ASSIMP_LINK /usr/local/lib/libassimp.5.dylib)

link_libraries(${OpenGL} ${GLEW_LINK} ${GLFW_LINK} ${ASSIMP_LINK})

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES "src/glad.c")

set(Third_PARTY_LIBS "-framework OpenGL" "-framework GLUT")

configure_file(configuration/root_directory.h.in configuration/root_directory.h)
include_directories(${CMAKE_BINARY_DIR}/configuration)

add_executable(OpenGL_test src/OpenGL_test.cpp ${SOURCE_FILES})
target_link_libraries(OpenGL_test ${Third_PARTY_LIBS})

add_executable(helloWindow src/helloWindow.cpp ${SOURCE_FILES})
target_link_libraries(helloWindow ${Third_PARTY_LIBS})

add_executable(helloTriangle src/helloTriangle.cpp ${SOURCE_FILES})
target_link_libraries(helloTriangle ${Third_PARTY_LIBS})

add_executable(helloRectangle src/helloRectangle.cpp ${SOURCE_FILES})
target_link_libraries(helloRectangle ${Third_PARTY_LIBS})

add_executable(hello_rectangle src/hello_rectangle.cpp ${SOURCE_FILES})
target_link_libraries(hello_rectangle ${Third_PARTY_LIBS})

add_executable(hello_two_triangles src/hello_two_triangles.cpp ${SOURCE_FILES})
target_link_libraries(hello_two_triangles ${Third_PARTY_LIBS})

add_executable(hello_change_triangle src/hello_change_triangle.cpp ${SOURCE_FILES})
target_link_libraries(hello_change_triangle ${Third_PARTY_LIBS})

add_executable(hello_container src/hello_container.cpp ${SOURCE_FILES})
target_link_libraries(hello_container ${Third_PARTY_LIBS})

add_executable(hello_disco_container src/hello_disco_container.cpp ${SOURCE_FILES})
target_link_libraries(hello_disco_container ${Third_PARTY_LIBS})

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

add_executable(hello_glsl_color src/hello_glsl_color.cpp ${SOURCE_FILES})
target_link_libraries(hello_glsl_color ${Third_PARTY_LIBS})

add_executable(hello_lighting_color src/hello_lighting_color.cpp ${SOURCE_FILES})
target_link_libraries(hello_lighting_color ${Third_PARTY_LIBS})

add_executable(hello_lighting_reflect src/hello_lighting_reflect.cpp ${SOURCE_FILES})
target_link_libraries(hello_lighting_reflect ${Third_PARTY_LIBS})

add_executable(hello_colors src/hello_colors.cpp ${SOURCE_FILES})
target_link_libraries(hello_colors ${Third_PARTY_LIBS})

add_executable(hello_shader_file src/hello_shader_file.cpp ${SOURCE_FILES})
target_link_libraries(hello_shader_file ${Third_PARTY_LIBS})

add_executable(hello_texture src/hello_texture.cpp ${SOURCE_FILES})
target_link_libraries(hello_texture ${Third_PARTY_LIBS})

add_executable(hello_face_texture src/hello_face_texture.cpp ${SOURCE_FILES})
target_link_libraries(hello_face_texture ${Third_PARTY_LIBS})

add_executable(hello_awsomeface src/hello_awsomeface_shader.cpp ${SOURCE_FILES})
target_link_libraries(hello_awsomeface ${Third_PARTY_LIBS})

add_executable(hello_multipleHappy src/hello_multiple_happy.cpp ${SOURCE_FILES})
target_link_libraries(hello_multipleHappy ${Third_PARTY_LIBS})

add_executable(hello_view_port src/hello_view_port.cpp ${SOURCE_FILES})
target_link_libraries(hello_view_port ${Third_PARTY_LIBS})

add_executable(hello_obj src/hello_obj.cpp ${SOURCE_FILES})
target_link_libraries(hello_obj ${Third_PARTY_LIBS})

add_executable(hello_plane_obj src/hello_plane_object.cpp ${SOURCE_FILES})
target_link_libraries(hello_plane_obj ${Third_PARTY_LIBS})

add_executable(hello_awsome_mix src/hello_awsome_mix.cpp ${SOURCE_FILES})
target_link_libraries(hello_awsome_mix ${Third_PARTY_LIBS})

add_executable(hello_rotation src/hello_rotation.cpp ${SOURCE_FILES})
target_link_libraries(hello_rotation ${Third_PARTY_LIBS})

add_executable(hello_coordinate src/hello_coordinate.cpp ${SOURCE_FILES})
target_link_libraries(hello_coordinate ${Third_PARTY_LIBS})

add_executable(hello_coordinate_answer src/hello_coordinate_answer.cpp ${SOURCE_FILES})
target_link_libraries(hello_coordinate_answer ${Third_PARTY_LIBS})

add_executable(hello_camera src/hello_camera.cpp ${SOURCE_FILES})
target_link_libraries(hello_camera ${Third_PARTY_LIBS})

add_executable(hello_model src/hello_model.cpp ${SOURCE_FILES})
target_link_libraries(hello_model ${Third_PARTY_LIBS})

add_executable(hello_camera_circle src/hello_camera_circle.cpp ${SOURCE_FILES})
target_link_libraries(hello_camera_circle ${Third_PARTY_LIBS})

add_executable(hello_moving src/hello_moving.cpp ${SOURCE_FILES})
target_link_libraries(hello_moving ${Third_PARTY_LIBS})

add_executable(hello_free_moving src/hello_free_moving.cpp ${SOURCE_FILES})
target_link_libraries(hello_free_moving ${Third_PARTY_LIBS})

add_executable(hello_mouse src/hello_mouse.cpp ${SOURCE_FILES})
target_link_libraries(hello_mouse ${Third_PARTY_LIBS})

add_executable(hello_mouse_moving src/hello_mouse_moving.cpp ${SOURCE_FILES})
target_link_libraries(hello_mouse_moving ${Third_PARTY_LIBS})

add_executable(hello_read_model src/hello_read_model.cpp ${SOURCE_FILES})
target_link_libraries(hello_read_model ${Third_PARTY_LIBS})

add_executable(hello_light src/hello_light.cpp ${SOURCE_FILES})
target_link_libraries(hello_light ${Third_PARTY_LIBS})

add_executable(hello_material src/hello_material.cpp ${SOURCE_FILES})
target_link_libraries(hello_material ${Third_PARTY_LIBS})

add_executable(hello_camera_circle_practice src/hello_camera_circle_practice.cpp ${SOURCE_FILES})
target_link_libraries(hello_camera_circle_practice ${Third_PARTY_LIBS})

add_executable(hello_z_buffer src/hello_z_buffer.cpp ${SOURCE_FILES})
target_link_libraries(hello_z_buffer ${Third_PARTY_LIBS})

add_executable(hello_parallel_light src/hello_parallel_light.cpp ${SOURCE_FILES})
target_link_libraries(hello_parallel_light ${Third_PARTY_LIBS})

add_executable(Polyhedron_Draw src/Polyhedron_Draw.cpp ${SOURCE_FILES})
target_link_libraries(Polyhedron_Draw ${Third_PARTY_LIBS})

add_executable(Quadric_Surface_Draw src/Quadric_Surface_Draw.cpp ${SOURCE_FILES})
target_link_libraries(Quadric_Surface_Draw ${Third_PARTY_LIBS})

add_executable(Spline_Approximation src/Spline_Approximation.cpp ${SOURCE_FILES})
target_link_libraries(Spline_Approximation ${Third_PARTY_LIBS})