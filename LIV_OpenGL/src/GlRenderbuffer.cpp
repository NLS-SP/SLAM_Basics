//
// Created by Robotics_qi on 2020/6/30.
//

#include <OpenGL/GlRenderbuffer.h>

namespace LIV_OpenGL{
    GlRenderbuffer::GlRenderbuffer(uint32_t width, uint32_t height, RENDERBUFFER_FORMAT fmt)
                                    :format_(static_cast<GLenum>(fmt)), width_(width),height_(height){
        glGenRenderbuffers(1, &id_);
        ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr){
            glDeleteRenderbuffers(1, ptr);
            delete ptr;
        });

        //allocate storage.
        glBindRenderbuffer(GL_RENDERBUFFER, id_);
        glRenderbufferStorage(GL_RENDERBUFFER, static_cast<GLenum>(fmt), width_, height_);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);

        CheckGLError();
    }

    void GlRenderbuffer::bind(){
        glBindRenderbuffer(GL_RENDERBUFFER, id_);
    }

    void GlRenderbuffer::release(){
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
    }

    uint32_t GlRenderbuffer::width() const{
        return width_;
    }

    uint32_t GlRenderbuffer::height() const{
        return height_;
    }
}