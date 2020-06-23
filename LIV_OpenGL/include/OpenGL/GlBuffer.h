//
// Created by Robotics_qi on 2020/6/22.
//

#ifndef LIV_OPENGL_GLBUFFER_H
#define LIV_OPENGL_GLBUFFER_H


#include <memory>
#include <string>
#include <vector>
#include <cassert>
#include <OpenGL/GlObject.h>

namespace LIV_OpenGL{
    class GlVertexArray;
    class TransformFeedback;

    //!@brief 告诉目标什么样的缓冲区被绑定
    enum class BUFFER_TARGET{
        ARRAY_BUFFER = GL_ARRAY_BUFFER,                                 // Vertex attribute
        ELEMENT_ARRAY_BUFFER = GL_ELEMENT_ARRAY_BUFFER,                 // Vertex array indices
        TRANSFORM_FEEDBACK_BUFFER = GL_TRANSFORM_FEEDBACK_BUFFER,       //
        TEXTURE_BUFFER = GL_TEXTURE_BUFFER                              // Texture attribute.
    };

    enum class BUFFER_USAGE{
        STREAM_DRAW = GL_STREAM_DRAW,
        STREAM_READ = GL_STREAM_READ,
        STREAM_COPY = GL_STREAM_COPY,
        STATIC_DRAW = GL_STATIC_DRAW,
        STATIC_READ = GL_STATIC_READ,
        STATIC_COPY = GL_STATIC_COPY,
        DYNAMIC_DRAW = GL_DYNAMIC_DRAW,
        DYNAMIC_READ = GL_DYNAMIC_READ,
        DYNAMIC_COPY = GL_DYNAMIC_COPY
    };

    template<class T>
    class GlBuffer : public GlObject{
    public:
        friend class GlVertexArray;
        friend class GlTransformFeedback;
        friend class GlTextureBuffer;

        //!@brief 初始化缓冲区，并说明缓冲区主要作用和初始化方式
        GlBuffer(BUFFER_TARGET target, BUFFER_USAGE usage);

        //!@brief 说明缓冲区大小
        size_t capacity() const;

        //!@brief 说明缓冲区中的元素个数
        size_t size() const;

        //!@brief 对缓冲区中的数据重新分配
        template<class dataStream>
        void assign(const std::vector<T, dataStream>& data);
        void assign(const T* data, const uint32_t n);
        void assign(const GlBuffer<T>& other);

        //!@brief 对缓冲区中的对象进行替换
        template<class dataStream>
        void replace(uint32_t offset, const std::vector<T, dataStream>& data);
        void replace(uint32_t offset, const T* data, const uint32_t n);

        void insert(uint32_t offset, const T& value);

        //!@brief 从缓冲区中读取数据
        void get(std::vector<T>& data);

        //!@brief 将缓冲区中的某一段数据[start, start + size]取出
        void get(std::vector<T>& data, uint32_t start, uint32_t size);

        //!@brief 将缓冲区中容量扩充为num_elements.
        void reserve(uint32_t num_elements);

        void resize(uint32_t new_size);

        BUFFER_TARGET target() const;
        BUFFER_USAGE usage()   const;

        void bind() override;
        void release() override;

        //!@brief 从指定缓冲区中拷贝数据
        void copyTo(GlBuffer<T>& other, uint32_t other_offset = 0);

        //!@brief 将其他缓冲区中的某一段复制过来
        void copyTo(uint32_t offset, uint32_t size, GlBuffer<T>& other, uint32_t other_offset);

        //!@brief 说明内存中所占空间的大小
        uint32_t memorySize() const{ return capacity_ * dataSize_; }

    protected:
        //!@brief 将其与定点缓冲变量绑定起来.
        GLuint bindTransparently();

        //!@brief 释放顶点缓冲对象
        void releaseTransparently(GLuint old_buffer);

        static GLuint boundBufferObject_;

        GLenum target_;
        size_t dataSize_{sizeof(T)};
        GLenum usage_;

        uint32_t capacity_{0};
        uint32_t size_{0};
    };

    template<class T>
    GLuint GlBuffer<T>::boundBufferObject_ = 0;

    template<class T>
    GlBuffer<T>::GlBuffer(LIV_OpenGL::BUFFER_TARGET target, LIV_OpenGL::BUFFER_USAGE usage):
                 target_(static_cast<GLenum>(target)), usage_(static_cast<GLenum>(usage)){
        glGenBuffers(1, &id_);
        ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr){
            glDeleteBuffers(1, ptr);
            delete ptr;
        });
    }

    template<class T>
    void GlBuffer<T>::bind(){
        boundBufferObject_ = id_;
        glBindBuffer(target_, id_);
    }

    template<class T>
    void GlBuffer<T>::release(){
        boundBufferObject_ = 0;
        glBindBuffer(target_, 0);
    }

    template<class T>
    BUFFER_TARGET GlBuffer<T>::target() const{
        return static_cast<BUFFER_TARGET>(target_);
    }

    template<class T>
    BUFFER_USAGE GlBuffer<T>::usage() const{
        return static_cast<BUFFER_USAGE>(usage_);
    }

    template<class T>
    template<class dataStream>
    void GlBuffer<T>::assign(const std::vector<T, dataStream>& data){
        assign(&data[0], data.size());
    }

    template<class T>
    void GlBuffer<T>::assign(const T* data, const uint32_t n){
        GLuint old_buffer = bindTransparently();

        if(capacity_ < n){
            glBufferData(target_, dataSize_ * n, 0, usage_);
            capacity_ = n;
        }

        glBufferSubData(target_, 0, dataSize_ * n, data);
        size_ = n;

        releaseTransparently(old_buffer);

        CheckGLError();
    }

    template<class T>
    template<class dataStream>
    void GlBuffer<T>::replace(uint32_t offset, const std::vector<T, dataStream>& data){
        replace(offset, &data[0], data.size());
    }

    template<class T>
    void GlBuffer<T>::replace(uint32_t offset, const T* data, const uint32_t n){
        if(offset >= size_) return;

        GLuint old_buffer = bindTransparently();
        glBufferSubData(target_, offset * dataSize_, dataSize_ * std::min(n, size_ - offset), data);
        releaseTransparently(old_buffer);
        CheckGLError();
    }

    template<class T>
    void GlBuffer<T>::insert(uint32_t offset, const T& value){
        replace(offset, &value, 1);
    }

    template<class T>
    void GlBuffer<T>::get(std::vector<T>& data){
        get(data, 0, size_);
    }

    template<class T>
    void GlBuffer<T>::get(std::vector<T>& data, uint32_t start, uint32_t size){
        size = std::min(size, size_ - start);

        data.clear();
        data.resize(size);

        GLuint old_buffer = bindTransparently();
        glGetBufferSubData(target_, start * dataSize_, size * dataSize_, &data[0]);
        releaseTransparently(old_buffer);
    }

    template<class T>
    void GlBuffer<T>::reserve(uint32_t num_elements){
        if(capacity_ >= num_elements) return;   // alread enough space.
        GLuint old_buffer = bindTransparently();

        std::vector<T> data(size_);

        // Have Copy Buffer here, not download & upload again
        if(size_ > 0) glGetBufferSubData(target_, 0, dataSize_ * size_, &data[0]);

        // Resize Buffer.
        glBufferData(target_, dataSize_ * num_elements, nullptr, static_cast<GLenum>(usage_));

        if(size_ > 0) glBufferSubData(target_, 0, dataSize_ * size_, &data[0]);

        capacity_ = num_elements;
        releaseTransparently(old_buffer);

        CheckGLError();
    }

    //!@brief 将指定的数据发送到缓存区中
    template<class T>
    void GlBuffer<T>::assign(const GlBuffer<T>& other){
        reserve(other.size());

        glBindBuffer(GL_COPY_READ_BUFFER, other.id_);
        glBindBuffer(GL_COPY_WRITE_BUFFER, id_);

        glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, dataSize_ * other.size_);

        glBindBuffer(GL_COPY_READ_BUFFER, 0);
        glBindBuffer(GL_COPY_WRITE_BUFFER, 0);

        size_ = other.size_;
    }

    template<class T>
    size_t GlBuffer<T>::size() const{
        return size_;
    }

    template<class T>
    size_t GlBuffer<T>::capacity() const{
        return capacity_;
    }

    template<class T>
    void GlBuffer<T>::resize(uint32_t new_size){
        reserve(new_size);
        size_ = new_size;
    }

    template<class T>
    GLuint GlBuffer<T>::bindTransparently(){
        if(boundBufferObject_ == id_) return id_;

        glBindBuffer(target_, id_);

        return boundBufferObject_;
    }

    template<class T>
    void GlBuffer<T>::releaseTransparently(GLuint old_buffer) {
        if(old_buffer == id_) return;   // nothing changed

        glBindBuffer(target_, old_buffer);
    }

    template<class T>
    void GlBuffer<T>::copyTo(GlBuffer<T>& other, uint32_t other_offset){
        copyTo(0, size_, other, other_offset);
    }

    template<class T>
    void GlBuffer<T>::copyTo(uint32_t offset, uint32_t size, GlBuffer<T>& other, uint32_t other_offset){
        glBindBuffer(GL_COPY_READ_BUFFER, id_);
        glBindBuffer(GL_COPY_WRITE_BUFFER, other.id());

        glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, offset * dataSize_, other_offset * dataSize_,
                            size * dataSize_);
        glBindBuffer(GL_COPY_READ_BUFFER, 0);
        glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
    }
}

#endif //LIV_OPENGL_GLBUFFER_H
