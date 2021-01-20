cmake_minimum_required(VERSION 2.8)
project(Vulkan_Study)

set(SOURCE_FILES src/main.cpp)
add_executable(VulkanEngine ${SOURCE_FILES})

include_directories(include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Finding and linking GLFW3
find_package(glfw3 REQUIRED)
if (glfw3_FOUND)
    include_directories(${glfw3_INCLUDE_DIRS})
    target_link_libraries (VulkanEngine glfw)
endif (glfw3_FOUND)

find_package (Vulkan)
if (Vulkan_FOUND)
    include_directories(${Vulkan_INCLUDE_DIRS})
    target_link_libraries (VulkanEngine ${Vulkan_LIBRARIES})
endif (Vulkan_FOUND)

set(Third_Party ${Vulkan_LIBRARIES} glfw)
add_executable(HelloTriangle src/HelloTriangleApplication.cpp)
target_link_libraries(HelloTriangle ${Third_Party})

add_executable(HelloCreateInstance src/hello_create_instance.cpp)
target_link_libraries(HelloCreateInstance ${Third_Party})

add_executable(HelloValidateLayer src/hello_validate_layer.cpp)
target_link_libraries(HelloValidateLayer ${Third_Party})