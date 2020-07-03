//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLTEXTURE_H
#define LIV_OPENGL_GLTEXTURE_H

#include <vector>
#include <OpenGL/GlObject.h>
#include <OpenGL/GlPixelFormat.h>
#include <OpenGL/GlTextureFormat.h>

namespace LIV_OpenGL{
    class GlFramebuffer;
    class GlTextureRectangle;

    enum class TEX_MIN_OP{
        LINEAR = GL_LINEAR,
        NEAREST = GL_NEAREST,
        NEAREST_MIPMAP_NEAREST = GL_NEAREST_MIPMAP_NEAREST,
        NEAREST_MIPMAP_LINEAR  = GL_NEAREST_MIPMAP_LINEAR,
        LINEAR_MIPMAP_NEAREST = GL_LINEAR_MIPMAP_NEAREST,
        LINEAR_MIPMAP_LINEAR  = GL_LINEAR_MIPMAP_LINEAR
    };

    enum class TEX_MAG_OP{
        LINEAR = GL_LINEAR,
        NEAREST = GL_NEAREST
    };

    enum class TEX_WRAP_OP{
        CLAMP_TO_EDGE = GL_CLAMP_TO_EDGE,
        CLAMP_TO_BORDER = GL_CLAMP_TO_BORDER,
        MIRRORED_REPEAT = GL_MIRRORED_REPEAT,
        REPEAT = GL_REPEAT,
        MIRROR_CLAMP_TO_EDGE = GL_MIRROR_CLAMP_TO_EDGE
    };

    enum class TEX_SWIZZLE{
        R = GL_RED,
        G = GL_GREEN,
        B = GL_BLUE,
        A = GL_ALPHA,
        ZERO = GL_ZERO,
        ONE = GL_ONE
    };

    class GlTexture : public GlObject{
    public:
        friend class GlFramebuffer;
        friend class GlTextureRectangle;

        //!@brief 创建一维的纹理图案[空纹理就用默认格式创建]
        GlTexture(uint32_t width, TEXTURE_FORMAT format = TEXTURE_FORMAT::RGB);
        //!@brief 创建一个二维的纹理图案【空纹理就用默认格式创建】
        GlTexture(uint32_t width, uint32_t height, TEXTURE_FORMAT format = TEXTURE_FORMAT::RGB);
        //!@brief 创建一个三维纹理图案【空纹理图案用默认格式创建】
        GlTexture(uint32_t width, uint32_t height, uint32_t depth, TEXTURE_FORMAT format = TEXTURE_FORMAT::RGB);

        ~GlTexture();

        //!@brief 纹理拷贝操作
        GlTexture clone() const;

        //!@brief 拷贝数据操作
        void copy(const GlTexture& other);

        //!@brief 数据绑定操作，这里是用来激活纹理单元，使其可以使用纹理属性
        void bind() override;
        void release() override;

        //!@brief 设置缩小过滤操作
        void setMinifyingOperation(TEX_MIN_OP minifyingOperation);
        //!@brief 设置放大过滤操作
        void setMagnifyingOperation(TEX_MAG_OP magnifyingOperation);


        //!@brief 设置纹理包裹操作
        //!@brief 一维纹理图像操作
        void setWrapOperation(TEX_WRAP_OP wrap_s);
        //!@brief 二维纹理图像操作
        void setWrapOperation(TEX_WRAP_OP wrap_s, TEX_WRAP_OP wrap_t);
        //!@brief 三维纹理图像操作
        void setWrapOperation(TEX_WRAP_OP wrap_s, TEX_WRAP_OP wrap_t, TEX_WRAP_OP wrap_r);

        void setTextureSwizzle(TEX_SWIZZLE red, TEX_SWIZZLE GREEN, TEX_SWIZZLE blue, TEX_SWIZZLE alpha);

        //!@brief 对纹理数据进行赋值
        template<typename T>
        void assign(PIXEL_FORMAT pixelFmt, PIXEL_TYPE type, T* data);

        //!@brief 对纹理数据大小重新复制
        //!@brief 一维图像的重新赋值
        void resize(uint32_t width);

        //!@brief 二维图像的重新赋值
        void resize(uint32_t width, uint32_t height);

        //!@brief 三维纹理模块重新赋值
        void resize(uint32_t width, uint32_t height, uint32_t depth);

        uint32_t width() const;
        uint32_t height() const;
        uint32_t depth() const;

        //!@brief 将纹理对象保存在某个文件中
        bool save(const std::string& filename) const;

        //!@brief 从某个文件中读取纹理对象
        static GlTexture loadTexture(const std::string& file_name);

        //!@brief 从给定的向量中读取到纹理特征
        template<typename T>
        void download(std::vector<T>& data) const;

        template<typename T>
        void download(T* ptr) const;

        template<typename T>
        void download(PIXEL_FORMAT pixelfmt, T* ptr) const;

        //!@brief 生成MipMap.
        void generateMipmaps();

    protected:
        GLuint bindTransparently() const;
        void releaseTransparently(GLuint old_id) const;

        void allocateMemory();

        static uint32_t numComponents(TEXTURE_FORMAT format);
        static GLuint boundTexture_;

        uint32_t width_{0}, height_{0}, depth_{0};
        GLenum target_;
        TEXTURE_FORMAT format_;
    };

    template<typename T>
    void GlTexture::assign(LIV_OpenGL::PIXEL_FORMAT pixelFmt, LIV_OpenGL::PIXEL_TYPE pixel_type, T *data) {
        GLuint old_id = bindTransparently();
        if(target_ == GL_TEXTURE_1D) glTexImage1D(target_, 0, static_cast<GLint>(format_), width_, 0, static_cast<GLenum>(pixelFmt),
                                                  static_cast<GLenum>(pixel_type), data);
        else if(target_ == GL_TEXTURE_2D) glTexImage2D(target_, 0, static_cast<GLint>(format_), width_, height_, 0, static_cast<GLenum>(pixelFmt),
                                                       static_cast<GLenum>(pixel_type), data);
        else if(target_ == GL_TEXTURE_3D) glTexImage3D(target_, 0, static_cast<GLint>(format_), width_, height_, depth_, 0, static_cast<GLenum>(pixelFmt),
                                                       static_cast<GLenum>(pixel_type), data);

        releaseTransparently(old_id);
    }
}



#endif //LIV_OPENGL_GLTEXTURE_H
