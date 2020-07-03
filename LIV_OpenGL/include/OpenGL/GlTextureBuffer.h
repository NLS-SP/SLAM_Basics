//
// Created by Robotics_qi on 2020/7/3.
//

#ifndef LIV_OPENGL_GLTEXTUREBUFFER_H
#define LIV_OPENGL_GLTEXTUREBUFFER_H
#include <OpenGL/GlBuffer.h>
#include <OpenGL/GlTextureFormat.h>

namespace LIV_OpenGL {
    //!@brief OpenGL的纹理对象
    // 这里是OpenGL保存纹理的地方
    class GlTextureBuffer : public GlObject {
    public:
        template<typename T>
        GlTextureBuffer(GlBuffer<T>& buffer, TEXTURE_FORMAT format)
            :format_(format){
                buffer_ = buffer.ptr_;
                glGenTextures(1, &id_);

                ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr){
                    glDeleteTextures(1, ptr);
                    delete ptr;
                });

                GLuint old_id = bindTransparently();
                GLint texFormat = static_cast<GLint>(format_);
                buffer.bind();
                glTexBuffer(GL_TEXTURE_BUFFER, texFormat, buffer.id());
                buffer.release();
                releaseTransparently(old_id);
                CheckGLError();
            }

            void bind() override{
                glBindTexture(GL_TEXTURE_BUFFER, id_);
                boundTexture_ = id_;
            }

            void release() override{
                glBindTexture(GL_TEXTURE_BUFFER, 0);
                boundTexture_ = 0;
            }

    protected:
        GLuint bindTransparently() const {
            if (boundTexture_ == id_) return id_;
            glBindTexture(GL_TEXTURE_BUFFER, id_);
            return boundTexture_;
        }

        void releaseTransparently(GLuint old_id) const {
            if (old_id == id_) return;

            glBindTexture(GL_TEXTURE_BUFFER, old_id);
        }

        static GLuint boundTexture_;
        std::shared_ptr<GLuint> buffer_;
        TEXTURE_FORMAT format_;
    };
}

#endif //LIV_OPENGL_GLTEXTUREBUFFER_H
