//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLPROGRAM_H
#define LIV_OPENGL_GLPROGRAM_H

#include <OpenGL/GlObject.h>
#include <OpenGL/GlShader.h>
#include <OpenGL/GlUniform.h>
#include <OpenGL/GlTransformFeedback.h>

#include <map>

namespace LIV_OpenGL{
    // OpenGL编程对象，用于附着多个着色器
    class GlProgram : public GlObject{
    public:
        //!@brief 创建单个程序帝乡
        GlProgram();

        //!@brief 将着色程序附着到编程上
        void attach(const GlShader& shader);

        //!@brief 将回溯绑定在程序上
        void attach(const GlTransformFeedback& feedback);

        //!@brief 将着色器程序连接在一起
        void link();

        //!@brief 使用程序开始进行渲染
        void bind() override;

        //!@brief 不再使用程序
        void release() override;

        //!@brief 设置单元格式
        void setUniform(const GlAbstractUniform& uniform);

    protected:
        static GLuint boundProgram_;
        bool linked_{false};

        std::map<SHADER_TYPE, GlShader> shaders_;
        std::vector<GlTransformFeedback> feedbacks_;

        GLuint bindTransparently();
        void releaseTransparently(GLuint oldProgram);
    };
}

#endif //LIV_OPENGL_GLPROGRAM_H
