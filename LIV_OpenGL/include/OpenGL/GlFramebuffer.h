//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLFRAMEBUFFER_H
#define LIV_OPENGL_GLFRAMEBUFFER_H

#include <map>
#include <OpenGL/GlTexture.h>
#include <OpenGL/GlTextureRectangle.h>
#include <OpenGL/GlRenderbuffer.h>

namespace LIV_OpenGL{
    enum class FRAMEBUFFER_TARGET{
        BOTH = GL_FRAMEBUFFER,
        READ = GL_READ_FRAMEBUFFER,
        DRAW = GL_DRAW_FRAMEBUFFER
    };

    enum class FRAMEBUFFER_ATTACHMENT{
        COLOR0 = GL_COLOR_ATTACHMENT0,
        COLOR1 = GL_COLOR_ATTACHMENT1,
        COLOR2 = GL_COLOR_ATTACHMENT2,
        COLOR3 = GL_COLOR_ATTACHMENT3,
        COLOR4 = GL_COLOR_ATTACHMENT4,
        COLOR5 = GL_COLOR_ATTACHMENT5,
        COLOR6 = GL_COLOR_ATTACHMENT5,

        DEPTH = GL_DEPTH_ATTACHMENT,
        STENCIL = GL_STENCIL_ATTACHMENT,
        DEPTH_STENCIL = GL_DEPTH_STENCIL_ATTACHMENT
    };

    //!@brief 图像缓冲区域
    class GlFramebuffer : public GlObject{
    public:
        GlFramebuffer(uint32_t width, uint32_t height, FRAMEBUFFER_TARGET target = FRAMEBUFFER_TARGET::BOTH);
        ~GlFramebuffer();

        // 将图像缓冲区与现在内容绑定
        void bind() override;
        void release() override;

        void attach(FRAMEBUFFER_ATTACHMENT target, GlTexture& texture);

        void attach(FRAMEBUFFER_ATTACHMENT target, GlRenderbuffer& buffer);

        void attach(FRAMEBUFFER_ATTACHMENT target, GlTextureRectangle &texture);

        bool valid() const;

        uint32_t width() const;

        uint32_t height() const;

        void resize(uint32_t width, uint32_t height);

    protected:
        GLuint bindTransparently();
        void releaseTransparently(GLuint old_id);

        static GLuint boundFramebuffer_;

        GLenum target_;
        bool valid_{false};
        uint32_t width_, height_;

        std::map<FRAMEBUFFER_ATTACHMENT, std::shared_ptr<GLuint> > attachments_;


    };
}

#endif //LIV_OPENGL_GLFRAMEBUFFER_H
