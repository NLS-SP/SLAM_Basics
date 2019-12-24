//
// Created by Robotics_qi on 2019/12/24.
//

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

void framebuffer_size_callback(GLFWwindow *window, int width, int height);

void pressInput(GLFWwindow *window);

int main()
{
    glfwInit();
    /**
     * @bief glfwInit 初始化GLFW
     *       glfwWindowHint 配置GLFW
     *                      告诉编译器用的GLFW版本是OpenGL3.3,并且使用的是核心模式
     */
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);    // Mac下代码需要使用
#endif


    GLFWwindow *window = glfwCreateWindow(1280, 796, "LearnOpenGL", NULL, NULL);
    if(window == NULL){
        std::cout << "Failed to create GLFW window" <<std::endl;
        glfwTerminate();
        return -1;
    }

    // 窗口创建完成后，让GLFW设置使窗口内容成为了主线程内容
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
        std::cout << "Failed to initialize the GLAD!" << std::endl;
        return -1;
    }

    // Render Loop
    while(!glfwWindowShouldClose(window)){


        pressInput(window);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // 检查有没有触发事件->通过触发更新窗口状态或调用对应的回调函数
        glfwPollEvents();
        glfwSwapBuffers(window);
    }

    glfwTerminate();
    return 0;

}

void pressInput(GLFWwindow *window){

    // Don't Understand when it is pressed ?
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS){
        std::cout << "Used the Quit Function!" << std::endl;
        glfwSetWindowShouldClose(window, true);
    }
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height){
    /**
    * @brief 前面俩个设置窗口左下角位置
    *        后面两个分辨显示窗口的维度（宽度和高度）
    * */
    glViewport(0, 0, width, height);
}