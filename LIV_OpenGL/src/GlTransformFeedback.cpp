//
// Created by Robotics_qi on 2020/7/2.
//

#include <OpenGL/GlException.h>
#include <OpenGL/GlTransformFeedback.h>

namespace LIV_OpenGL{
    GlTransformFeedback::GlTransformFeedback()
                         :bound_(std::make_shared<bool>(false)), linked_(std::make_shared<bool>(false)){
#if __GL_VERSION >= 400L
        glGenTransformFeedbacks(1, &id_);
        ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr){
            glDeleteTransformFeedbacks(1, ptr);
            delete ptr;
        });
#endif
    }

    void GlTransformFeedback::bind(){
        if(!*linked_) throw GlTransformFeedbackError("Transfrom feedback not linked with any program!");
#if __GL_VERSION >= 400L
        glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, id_);
#endif

        // bind buffers.
        for(uint32_t i = 0; i < buffers_.size(); ++i)
            glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, i, *(buffers_[i].second));
        CheckGLError();
        *bound_ = true;
    }

    void GlTransformFeedback::begin(LIV_OpenGL::TRANSFORM_FEEDBACK_MODE mode) {
        if(!*bound_) throw GlTransformFeedbackError("Trasnform feedback must be bound using bind() before calling being()");
        countquery_.begin();
        glBeginTransformFeedback(static_cast<GLenum>(mode));

        CheckGLError();
    }

    uint32_t GlTransformFeedback::end(){
        glEndTransformFeedback();
        glFinish();
        countquery_.end();
        return (uint32_t)countquery_;
    }

#if __GL_VERSION >= 400L
void GlTransformFeedback::pause(){
        glPauseTransformFeedback();
    }

    void GlTransformFeedback::resume(){
        glResumeTransformFeedback();
    }

    void GlTransformFeedback::draw(GLenum mode) const{
        glDrawTransformFeedback(mode, id_);
    }
#endif

    void GlTransformFeedback::release(){
#if __GL_VERSION >= 400L
        glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);
#endif
        // bind buffers.
        for(uint32_t i = 0; i < buffers_.size(); ++i)
            glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, i, 0);
        *bound_ = false;
    }

    void GlTransformFeedback::registerVaryings(GLuint program_id) {
        GLenum bufferMode = (buffers_.size() == 1) ? GL_INTERLEAVED_ATTRIBS : GL_SEPARATE_ATTRIBS;
        bool addNextBuffer = false;
        std::string nextBuffer = "gl_NextBuffer";

        for(auto&p : buffers_){
            if(p.first.size() > 1 && buffers_.size() > 1){
#if __GL_VERSION >= 400L
                bufferMode = GL_INTERLEAVED_ATTRIBS;
                addNextBuffer = true;
                break;
#else
                throw GlTransformFeedbackError("Multiple interleaved buffers using gl_NextBuffer only supported beginning with GL version 4.0");
#endif
            }
        }

        std::vector<std::string> varyings_stack;
        for(auto& p : buffers_){
            for(uint32_t i = 0; i < p.first.size(); ++i)
                varyings_stack.push_back(p.first[i]);
            if(addNextBuffer) varyings_stack.push_back(nextBuffer);
        }

        // 2. set varying.
        const char* vars[varyings_stack.size()];
        uint32_t count = 0;
        for(const auto& v : varyings_stack)
            vars[count++] = v.c_str();

        glTransformFeedbackVaryings(program_id, varyings_stack.size(), vars, bufferMode);
        *linked_ = true;
    }
}