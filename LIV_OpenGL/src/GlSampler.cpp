//
// Created by Robotics_qi on 2020/6/29.
//

#include <OpenGL/GlSampler.h>

namespace LIV_OpenGL{
    GlSampler::GlSampler() {
        glGenSamplers(1, &id_);
        ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr){
            glDeleteSamplers(1, ptr);
            delete ptr;
        });
    }

    //!@brief 使用采样对特殊的纹理单元进行绑定
    void GlSampler::bind(uint32_t textureUnitId){
        glBindSampler(static_cast<GLuint>(textureUnitId), id_);
    }

    void GlSampler::release(uint32_t textureUnitId){
        glBindSampler(static_cast<GLuint>(textureUnitId), 0);
    }

    void GlSampler::bind(){}
    void GlSampler::release(){}

    void GlSampler::setMinifyingOperation(LIV_OpenGL::TEX_MIN_OP minifyingOperation) {
        glSamplerParameteri(id_, GL_TEXTURE_MIN_FILTER, static_cast<GLenum>(minifyingOperation));
    }

    void GlSampler::setMagnifyingOperation(LIV_OpenGL::TEX_MAG_OP magnifyingOperation) {
        glSamplerParameteri(id_, GL_TEXTURE_MAG_FILTER, static_cast<GLenum>(magnifyingOperation));
    }

    void GlSampler::setWrapOperation(LIV_OpenGL::TEX_WRAP_OP wrap_s) {
        glSamplerParameteri(id_, GL_TEXTURE_WRAP_S, static_cast<GLenum>(wrap_s));
    }

    void GlSampler::setWrapOperation(LIV_OpenGL::TEX_WRAP_OP wrap_s, TEX_WRAP_OP wrap_t){
        glSamplerParameteri(id_, GL_TEXTURE_WRAP_S, static_cast<GLenum>(wrap_s));
        glSamplerParameteri(id_, GL_TEXTURE_WRAP_T, static_cast<GLenum>(wrap_t));
    }

    void GlSampler::setWrapOperation(LIV_OpenGL::TEX_WRAP_OP wrap_s, LIV_OpenGL::TEX_WRAP_OP wrap_t,
                                     LIV_OpenGL::TEX_WRAP_OP wrap_r) {
        glSamplerParameteri(id_, GL_TEXTURE_WRAP_S, static_cast<GLenum>(wrap_s));
        glSamplerParameteri(id_, GL_TEXTURE_WRAP_T, static_cast<GLenum>(wrap_t));
        glSamplerParameteri(id_, GL_TEXTURE_WRAP_R, static_cast<GLenum>(wrap_r));
    }
}
