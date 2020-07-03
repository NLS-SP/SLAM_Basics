//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLVERTEXARRAY_H
#define LIV_OPENGL_GLVERTEXARRAY_H

#include <map>
#include <memory>
#include <OpenGL/GlObject.h>
#include <OpenGL/GlBuffer.h>

namespace LIV_OpenGL{
    //!@brief vertex的数据属性类别
    enum class ATTRIBUTE_TYPE{
        BYTE = GL_BYTE,
        UNSIGNED_BYTE = GL_UNSIGNED_BYTE,
        SHORT = GL_SHORT,
        UNSIGNED_SHORT = GL_UNSIGNED_SHORT,
        INT = GL_INT,
        UNSIGNED_INT = GL_UNSIGNED_INT,
        HALF_FLOAT = GL_HALF_FLOAT,
        FLOAT = GL_FLOAT,
        DOUBLE = GL_DOUBLE,
        FIXED = GL_FIXED,
        INT_2_10_10_10_REV = GL_INT_2_10_10_10_REV,
        UNSIGNED_INT_2_10_10_10_REV = GL_UNSIGNED_INT_2_10_10_10_REV,
        UNSIGNED_INT_10F_11F_11F_REV = GL_UNSIGNED_INT_10F_11F_11F_REV
    };

    //!@brief OpenGL中的顶点数据类型
    class GlVertexArray : public GlObject{
    public:
        GlVertexArray();
        ~GlVertexArray();

        //!@brief 顶点数据对象绑定
        void bind() override;
        //!@brief 顶点数据对象释放
        void release() override;

        template<typename T>
        void setVertexAttribute(uint32_t idx, GlBuffer<T>& buffer, int32_t size, ATTRIBUTE_TYPE type, bool normalized,
                                uint32_t stride, GLvoid* offset);

        void enableVertexAttribute(uint32_t idx);

        void disableVertexAttribute(uint32_t idx);


    protected:
        //!@brief 对顶点数据绑定
        GLuint bindTransparently();
        void releaseTransparently(GLuint old_vao);

        static GLuint boundVAO_;

        struct VertexAttributeState{
        public:
            GlObject* buffer{0};
            bool initialized{false};
            bool enabled{false};
        };
        std::map<uint32_t, std::shared_ptr<GLuint> > vertexBuffers_;
    };

    template<typename T>
    void GlVertexArray::setVertexAttribute(uint32_t idx, LIV_OpenGL::GlBuffer<T> &buffer, int32_t numComponents,
                                           LIV_OpenGL::ATTRIBUTE_TYPE type, bool normalized, uint32_t stride_in_bytes,
                                           GLvoid *offset) {

        vertexBuffers_[idx] = buffer.ptr_;
        GLuint old_vao = bindTransparently();
        buffer.bind();

        if(type == ATTRIBUTE_TYPE::INT || type == ATTRIBUTE_TYPE::UNSIGNED_INT){
            glVertexAttribIPointer(static_cast<GLuint>(idx), static_cast<GLint>(numComponents), static_cast<GLenum>(type),
                                   static_cast<GLuint>(stride_in_bytes),  offset);
        }else{
            glVertexAttribPointer(static_cast<GLuint>(idx), static_cast<GLint>(numComponents), static_cast<GLenum>(type),
                                  static_cast<GLboolean>(normalized), static_cast<GLuint>(stride_in_bytes), offset);
        }

        glEnableVertexAttribArray(static_cast<GLuint>(idx));

        releaseTransparently(old_vao);
        buffer.release();
        CheckGLError();
    }
}



#endif //LIV_OPENGL_GLVERTEXARRAY_H
