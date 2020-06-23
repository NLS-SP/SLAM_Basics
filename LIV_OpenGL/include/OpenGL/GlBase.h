//
// Created by Robotics_qi on 2020/6/22.
//

#ifndef LIV_OPENGL_GLBASE_H
#define LIV_OPENGL_GLBASE_H



#include <iostream>
#include <cassert>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <OpenGL/GlException.h>

// if no version is specified, fall back to OpenGL version 3.30
#ifndef __GL_VERSION
#define __GL_VERSION 330L
#endif

namespace LIV_OpenGL{
    inline void _CheckGLError(const char* configure_file, const int nLine){
        GLenum glError = glGetError();
        if(glError != GL_NO_ERROR){
            std::cerr << "OpenGL Error: " << gluErrorString(glError) << "(" << glError << ")" << std::endl;
            std::cerr << "In: " << configure_file << " on Line: " << nLine << std::endl;
            throw std::runtime_error("OpenGL error detected");
        }
    }

    static bool glewInitialized = false;

#ifdef NDEBUG
#define CheckGlError() (static_cast<void>(0))
#else
#define CheckGLError()(LIV_OpenGL::_CheckGLError(__FILE__, __LINE__ ));
#endif

    inline void initializeGLEW(){
        if(!glewInitialized){
            glewExperimental = GL_TRUE;
            GLenum err = glewInit();

            if(err != GLEW_OK){
                std::cerr << glewGetErrorString(err) << std::endl;
                throw std::runtime_error("Failed to initialize GLEW.");
            }
        }

        std::cout << "GLEW initialized." << std::endl;
        glewInitialized = true;
        int32_t version[2];
        glGetIntegerv(GL_MAJOR_VERSION, &version[0]);
        glGetIntegerv(GL_MINOR_VERSION, &version[1]);
        std::cout << "OpenGL Context Version: " << version[0] << "." << version[1] << std::endl;
        std::cout << "OpenGL Vendor   String: " << glGetString(GL_VENDOR) << std::endl;
        std::cout << "OpenGL Renderer String: " << glGetString(GL_RENDERER) << std::endl;

        // consume here any OpenGL error and reset to NO_GL_ERROR:
        glGetError();
    }


#define PRINT_VALUE(CMD) std::cout << #CMD << " = " << CMD << std::endl;
#define PRINT_CLASS_VALUE(CLASS, CMD) std::cout << #CLASS << "::" << #CMD << " = " << CMD << std::endl;
}


#endif //LIV_OPENGL_GLBASE_H
