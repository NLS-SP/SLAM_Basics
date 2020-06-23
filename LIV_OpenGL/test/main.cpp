//
// Created by Robotics_qi on 2020/6/22.
//

#include <gtest/gtest.h>
#include <OpenGL/GlBase.h>
#include <OpenGL/GlBuffer.h>
#include <random>
#include <Eigen/Dense>

TEST(BufferTest, initTest){
    LIV_OpenGL::GlBuffer<float> buffer(LIV_OpenGL::BUFFER_TARGET::ARRAY_BUFFER, LIV_OpenGL::BUFFER_USAGE::DYNAMIC_DRAW);
    buffer.bind();
    buffer.release();
    ASSERT_TRUE(glIsBuffer(buffer.id()));
}



GTEST_API_ int main(int argc, char** argv)
{
    glewExperimental = true; // Needed for core profile
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        return -1;
    }
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL

// Open a window and create its OpenGL context
    GLFWwindow* window; // (In the accompanying source code, this variable is global for simplicity)
    window = glfwCreateWindow( 1024, 768, "Tutorial 01", NULL, NULL);
    if( window == NULL ){
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window); // Initialize GLEW
    LIV_OpenGL::initializeGLEW();
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
