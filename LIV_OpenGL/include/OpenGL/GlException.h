//
// Created by Robotics_qi on 2020/6/22.
//

#ifndef LIV_OPENGL_GLEXCEPTION_H
#define LIV_OPENGL_GLEXCEPTION_H

#include <stdexcept>

namespace LIV_OpenGL {
    //!@brief 创建着色器过程中的错误查询
    class GlShaderError : public std::runtime_error {
    public:
        GlShaderError(const std::string &msg);
    };

    class GlProgramError : public std::runtime_error {
    public:
        GlProgramError(const std::string &msg);
    };

    class GlOffScreenContextError : public std::runtime_error {
    public:
        GlOffScreenContextError(const std::string &msg);
    };

    class GlVertexArrayError : public std::runtime_error {
    public:
        GlVertexArrayError(const std::string &msg);
    };

    class GlTextureError : public std::runtime_error {
    public:
        GlTextureError(const std::string &msg);
    };

    class GlFramebufferError : public std::runtime_error {
    public:
        GlFramebufferError(const std::string &msg);
    };

    class GlTransformFeedbackError : public std::runtime_error {
    public:
        GlTransformFeedbackError(const std::string &msg);
    };

    class GlQueryError : public std::runtime_error {
    public:
        GlQueryError(const std::string &msg);
    };

    class GlTextureRectangleError : public std::runtime_error {
    public:
        GlTextureRectangleError(const std::string &msg);
    };
}

#endif //LIV_OPENGL_GLEXCEPTION_H
