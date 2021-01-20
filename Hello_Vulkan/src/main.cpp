//
// Created by Robotics_qi on 2020/10/13.
//

#include <GLFW/glfw3.h>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <iostream>
#include <vulkan/vulkan.h>

int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(800, 600, "Vulkan Window", nullptr, nullptr);
    uint32_t extensionCount = 0;
    vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);
    std::cout << extensionCount << " extensions supported" << std::endl;
    glm::mat4 matrix;
    glm::vec4 vec;
    auto test = matrix * vec;
    while(!glfwWindowShouldClose(window))
        glfwPollEvents();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}