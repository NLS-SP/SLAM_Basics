//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLRENDERBUFFER_H
#define LIV_OPENGL_GLRENDERBUFFER_H

#include <OpenGL/GlObject.h>

namespace LIV_OpenGL {


    class GlFramebuffer;

    enum class RENDERBUFFER_FORMAT{
        RGBA = GL_RGBA,
        RGB = GL_RGB,
        RG = GL_RG,
        R = GL_R,
        DEPTH = GL_DEPTH,
        DEPTH_STENCIL = GL_DEPTH24_STENCIL8
    };

    //!@brief 用来渲染的OpenGL对象
    class GlRenderbuffer : public GlObject {
    public:
        friend class GlFramebuffer;

        GlRenderbuffer(uint32_t width, uint32_t height, RENDERBUFFER_FORMAT fmt = RENDERBUFFER_FORMAT::RGB);

        void bind() override;
        void release() override;

        uint32_t width() const;
        uint32_t height() const;

    protected:
        GLenum format_;
        uint32_t width_, height_;
    };
}
#endif //LIV_OPENGL_GLRENDERBUFFER_H
