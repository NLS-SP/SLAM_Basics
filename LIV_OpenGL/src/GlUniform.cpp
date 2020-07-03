//
// Created by Robotics_qi on 2020/7/1.
//

#include <OpenGL/GlUtil.h>
#include <OpenGL/GlUniform.h>
#include <OpenGL/GlException.h>

namespace LIV_OpenGL{
    template<>
    void GlUniform<Eigen::Matrix4f>::bind(GLuint program_id) const{
        GLint loc = glGetUniformLocation(program_id, name_.c_str());
        glUniformMatrix4fv(loc, 1, GL_FALSE, data_.data());
    }

    template<>
    void GlUniform<Eigen::Vector4f>::bind(GLuint program_id) const{
        GLint loc = glGetUniformLocation(program_id, name_.c_str());
        glUniform4fv(loc, 1, data_.data());
    }

    template<>
    void GlUniform<Eigen::Vector3f>::bind(GLuint program_id) const{
        GLint loc = glGetUniformLocation(program_id, name_.c_str());
        glUniform3fv(loc, 1, data_.data());
    }

    template<>
    void GlUniform<vec4>::bind(GLuint program_id) const{
        GLint loc = glGetUniformLocation(program_id, name_.c_str());
        glUniform4fv(loc, 1, &data_.x);
    }

    template<>
    void GlUniform<vec3>::bind(GLuint program_id) const{
        GLint loc = glGetUniformLocation(program_id, name_.c_str());
        glUniform3fv(loc, 1, &data_.x);
    }

    template<>
    void GlUniform<vec2>::bind(GLuint program_id) const{
        GLint loc = glGetUniformLocation(program_id, name_.c_str());
        glUniform2fv(loc, 1, &data_.x);
    }

    template<>
    void GlUniform<int32_t>::bind(GLuint program_id) const{
        GLint loc = glGetUniformLocation(program_id, name_.c_str());
        glUniform1i(loc, static_cast<GLint>(data_));
    }

    template<>
    void GlUniform<uint32_t>::bind(GLuint program_id) const{
        GLint loc = glGetUniformLocation(program_id, name_.c_str());
        glUniform1ui(loc, static_cast<GLuint>(data_));
    }

    template<>
    void GlUniform<bool>::bind(GLuint program_id) const{
        GLint loc = glGetUniformLocation(program_id, name_.c_str());
        glUniform1i(loc, static_cast<GLint>(data_));
    }

    template<>
    void GlUniform<float>::bind(GLuint program_id) const{
        GLint loc = glGetUniformLocation(program_id, name_.c_str());
        glUniform1f(loc, static_cast<GLfloat>(data_));
    }
}
