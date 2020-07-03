//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLTEXTUREFORMAT_H
#define LIV_OPENGL_GLTEXTUREFORMAT_H

#include <OpenGL/GlObject.h>

namespace LIV_OpenGL{
    /*****************
     * !@brief 这里主要用来定义纹理格式
     *****************/
    enum class TEXTURE_FORMAT{
        // 基础纹理格式
        RGBA  = GL_RGBA,
        RGB   = GL_RGB,
        RG    = GL_RG,
        R     = GL_RED,
        DEPTH = GL_DEPTH,
        DEPTH_STENCIL = GL_DEPTH24_STENCIL8,

        // 一些针对32位的纹理格式
        R_INTEGER = GL_R32I,
        RG_INTEGER = GL_RG32I,
        RGB_INTEGER = GL_RGB32I,
        RGBA_INTEGER = GL_RGBA32I,

        R_FLOAT = GL_R32F,
        RG_FLOAT = GL_RG32F,
        RGB_FLOAT = GL_RGB32F,
        RGBA_FLOAT = GL_RGBA32F
    };
}

#endif //LIV_OPENGL_GLTEXTUREFORMAT_H
