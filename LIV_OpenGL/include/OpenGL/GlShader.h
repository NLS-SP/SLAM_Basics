//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLSHADER_H
#define LIV_OPENGL_GLSHADER_H

#include <string>
#include <memory>
#include <vector>
#include <OpenGL/GlObject.h>

namespace LIV_OpenGL{
    enum class SHADER_TYPE{
        VERTEX_SHADER = GL_VERTEX_SHADER,
        TESS_CONTROL_SHADER = GL_TESS_CONTROL_SHADER,
        TESS_EVALUTION_SHADER = GL_TESS_EVALUATION_SHADER,
        GEOMETRY_SHADER = GL_GEOMETRY_SHADER,
        FRAGMENT_SHADER = GL_FRAGMENT_SHADER,
        COMPUTE_SHADER = GL_COMPUTE_SHADER
    };

    /********************
     * !@brief 主要针对OpenGL中的着色器(Shader)对象
     * 这里参考Pangolin的做法，OpenGL中的Shader一般通过从文件中读取创建
     ********************/
    class GlShader : public GlObject{
    public:
        friend class GlProgram;

        //!@brief 初始化操作,使用文件创建
        GlShader(const SHADER_TYPE& type, const std::string& source, bool useCache = false);

        //!@brief 返回说明当前Shader的类型
        SHADER_TYPE type() const;

        //!@brief 从文件中创建Shader对象
        static GlShader fromeFile(const SHADER_TYPE& type, const std::string& filename);

        //!@brief 从缓冲区中进行Shader创建
        static GlShader fromeCache(const SHADER_TYPE& type, const std::string& filename);

    protected:

        void bind() override;
        void release() override;

        SHADER_TYPE  type_;

        static std::string getCachedSource(const std::string& filename);
        static std::string readSource(const std::string& file_name);

        std::string preProcess(const std::string& source);

        struct ATTRIBUTE{
        public:
            std::string type;
            std::string name;
        };

        std::string filename;
        bool useCache_{false};

        std::vector<ATTRIBUTE> inAttribs_;
        std::vector<ATTRIBUTE> outAttribs_;
    };

}


#endif //LIV_OPENGL_GLSHADER_H
