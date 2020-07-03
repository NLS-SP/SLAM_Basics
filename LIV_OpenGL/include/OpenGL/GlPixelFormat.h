//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLPIXELFORMAT_H
#define LIV_OPENGL_GLPIXELFORMAT_H

#include <OpenGL/GlObject.h>
/** **************************
 *  !@brief 像素格式和像素类型定义
 ** **************************/


namespace LIV_OpenGL{
    enum class PIXEL_FORMAT{
        // 像素颜色存储在[0.0, 1.0]区间内
        R = GL_RED,
        RG = GL_RG,
        RGB = GL_RGB,
        BGR = GL_BGR,
        RGBA = GL_RGBA,
        BGRA = GL_BGRA,
        // 像素颜色存储在[0, 2^B-1]中，这里B是字节大小
        R_INTEGER = GL_RED_INTEGER,
        RG_INTEGER = GL_RG_INTEGER,
        RGB_INTEGER = GL_RGB_INTEGER,
        BGR_INTEGER = GL_BGRA_INTEGER,
        RGBA_INTEGER = GL_RGBA_INTEGER,
        BGRA_INTEGER = GL_BGRA_INTEGER,
        DEPTH = GL_DEPTH_COMPONENT,
        DEPTH_STENCIL = GL_DEPTH24_STENCIL8,
        STENCIL = GL_STENCIL_INDEX
    };

    enum class PIXEL_TYPE{
        UNSIGNED_BYTE = GL_UNSIGNED_BYTE,
        BYTE = GL_BYTE,
        UNSIGEND_SHORT = GL_UNSIGNED_SHORT,
        SHORT = GL_SHORT,
        UNSIGEND_INT = GL_UNSIGNED_INT,
        INT = GL_INT,
        HALF_FLOAT = GL_HALF_FLOAT,
        FLOAT = GL_FLOAT
    };
}

#endif //LIV_OPENGL_GLPIXELFORMAT_H
