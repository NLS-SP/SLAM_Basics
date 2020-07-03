//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLTEXTURERECTANGLE_H
#define LIV_OPENGL_GLTEXTURERECTANGLE_H

#include <vector>
#include <OpenGL/GlObject.h>
#include <OpenGL/GlPixelFormat.h>
#include <OpenGL/GlTextureFormat.h>

namespace LIV_OpenGL{
    class GlTexture;

    enum class TEX_RECT_MIN_OP{LINEAR = GL_LINEAR, NEAREST = GL_NEAREST};
    enum class TEX_RECT_MAG_OP{LINEAR = GL_LINEAR, NEAREST = GL_NEAREST};
    enum class TEX_RECT_WRAP_OP{
        CLAMP_TO_EDGE = GL_CLAMP_TO_EDGE,
        CLAMP_TO_BORDER = GL_CLAMP_TO_BORDER,
        MIRRORED_REPEAT = GL_MIRRORED_REPEAT,
        MIRROR_CLAMP_TO_EDGE = GL_MIRROR_CLAMP_TO_EDGE
    };
    enum class TEX_RECT_SWIZZLE{
        R = GL_RED,
        G = GL_GREEN,
        B = GL_BLUE,
        A = GL_ALPHA,
        ZERO = GL_ZERO,
        ONE = GL_ONE
    };

    class GlTextureRectangle : public GlObject{
    public:
        friend class GlFramebuffer;

        //!@brief 创建给定大小的长方形纹理
        GlTextureRectangle(uint32_t width, uint32_t height, TEXTURE_FORMAT);
        ~GlTextureRectangle();

        //!@brief 对纹理进行拷贝
        GlTextureRectangle clone() const;

        //!@brief 从其他纹理区域进行数据拷贝
        void copy(const GlTexture& other);

        //!@brief 将数据拷贝到这个纹理区域
        void copy(const GlTextureRectangle& other);

        //!@brief 对纹理区域进行绑定和激活
        void bind() override ;
        void release() override;

        //!@brief 设置过滤方法
        void setMinifyingOperation(TEX_RECT_MIN_OP minifyingOperation);
        void setMagnifyingOperation(TEX_RECT_MAG_OP magnifyingOperation);

        //!@brief 设置包裹区域
        void setWrapOperation(TEX_RECT_WRAP_OP wrap_s, TEX_RECT_WRAP_OP wrap_t);

        void setTextureSwizzle(TEX_RECT_SWIZZLE red, TEX_RECT_SWIZZLE green, TEX_RECT_SWIZZLE blue, TEX_RECT_SWIZZLE alpha);

        //!@brief 对纹理区域进行数据分配
        template<typename T>
        void assign(PIXEL_FORMAT pixelfmt, PIXEL_TYPE type, T* data);

        void resize(uint32_t width, uint32_t height);

        uint32_t width() const;
        uint32_t height() const;

        bool save(const std::string& file_name) const;

        static GlTextureRectangle loadTexture(const std::string& file_name);

        template<typename T>
        void download(std::vector<T>& data) const;

        template<typename T>
        void download(T* ptr) const;

        template<typename T>
        void download(PIXEL_FORMAT pixelfmt, T* ptr) const;

    protected:
        GLuint bindTransparently() const;
        void releaseTransparently(GLuint old_id) const;

        void allocateMemory();

        static uint32_t numComponents(TEXTURE_FORMAT fmt);
        static GLuint boundTexture_;

        uint32_t width_, height_;
        TEXTURE_FORMAT format_;
    };

    template<typename T>
    void GlTextureRectangle::assign(LIV_OpenGL::PIXEL_FORMAT pixelfmt, LIV_OpenGL::PIXEL_TYPE pixeltype, T *data) {
        GLuint old_id = bindTransparently();

        glTexImage2D(GL_TEXTURE_RECTANGLE, 0, static_cast<GLint>(format_), width_, height_, 0, static_cast<GLenum>(pixelfmt),
                    static_cast<GLenum>(pixeltype), data);

        releaseTransparently(old_id);
    }
}

#endif //LIV_OPENGL_GLTEXTURERECTANGLE_H
