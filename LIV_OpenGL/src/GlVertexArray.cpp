//
// Created by Robotics_qi on 2020/6/30.
//

#include <cassert>
#include <OpenGL/GlException.h>
#include <OpenGL/GlVertexArray.h>

namespace LIV_OpenGL{
    GLuint GlVertexArray::boundVAO_ = 0;

    GlVertexArray::GlVertexArray(){
        glGenVertexArrays(1, &id_);
        ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr){
            glDeleteVertexArrays(1, ptr);
            delete ptr;
        });
    }

    GlVertexArray::~GlVertexArray(){}

    void GlVertexArray::bind(){
        assert((boundVAO_ == 0 || boundVAO_ == id_) && "Other vertex array object still active？" );
        boundVAO_ = id_;
        glBindVertexArray(id_);
    }

    void GlVertexArray::release(){
        assert(boundVAO_ == id_ && "Different vertex array object bound in between?");
        boundVAO_ = 0;
        glBindVertexArray(0);
    }

    void GlVertexArray::enableVertexAttribute(uint32_t idx){
        GLuint old_vao = bindTransparently();
        glEnableVertexAttribArray(static_cast<GLuint>(idx));
        releaseTransparently(old_vao);
    }

    void GlVertexArray::disableVertexAttribute(uint32_t idx) {
        GLuint old_vao = bindTransparently();
        glDisableVertexAttribArray(static_cast<GLuint>(idx));
        releaseTransparently(old_vao);
    }

    GLuint GlVertexArray::bindTransparently() {
        if(boundVAO_ == id_) return id_;
        glBindVertexArray(id_);
        return boundVAO_;
    }

    void GlVertexArray::releaseTransparently(GLuint old_vao){
        if(old_vao == id_) return;
        glBindVertexArray(old_vao);
    }
}