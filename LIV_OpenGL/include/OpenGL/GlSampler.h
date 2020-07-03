//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLSAMPLER_H
#define LIV_OPENGL_GLSAMPLER_H

#include <OpenGL/GlObject.h>
#include <OpenGL/GlTexture.h>

namespace LIV_OpenGL{
    class GlSampler : public GlObject{
    public:
        GlSampler();

        //!@brief 使用采样器来对纹理单元进行抽取
        void bind(uint32_t textureUnitId);

        //!@brief 不使用抽样器来做
        void release(uint32_t textureUnitId);

        //!@brief 对小物体单元进行过滤操作
        void setMinifyingOperation(TEX_MIN_OP minifyingOperation);
        //!@brief 对大物体单元进行滤波操作
        void setMagnifyingOperation(TEX_MAG_OP magnifyingOperation);

        //!@brief 设置包裹操作
        void setWrapOperation(TEX_WRAP_OP wrap_s);
        void setWrapOperation(TEX_WRAP_OP wrap_s, TEX_WRAP_OP wrap_t);
        void setWrapOperation(TEX_WRAP_OP wrap_s, TEX_WRAP_OP wrap_t, TEX_WRAP_OP wrap_r);

    protected:
        void bind() override;
        void release() override;

        uint32_t boundUnit_{0};

    };
}

#endif //LIV_OPENGL_GLSAMPLER_H
