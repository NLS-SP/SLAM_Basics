//
// Created by Robotics_qi on 2020/6/29.
//

#ifndef LIV_OPENGL_GLQUERY_H
#define LIV_OPENGL_GLQUERY_H

#include <OpenGL/GlObject.h>

namespace LIV_OpenGL{
    enum class QUERY_TARGET{
        SAMPLES_PASSED = GL_SAMPLES_PASSED,
        ANY_SAMPLES_PASSED = GL_ANY_SAMPLES_PASSED,
        ANY_SAMPLES_PASSED_CONSERVATIVE = GL_ANY_SAMPLES_PASSED_CONSERVATIVE,
        PRIMITIVES_GENERATED = GL_PRIMITIVES_GENERATED,
        TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN = GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN,
        TIME_ELAPSED = GL_TIME_ELAPSED
    };

    /*********************************
     * !@brief 这里是OpenGL的查询对象函数
     *********************************/
    class GlQuery : public GlObject{
    public:
        GlQuery(QUERY_TARGET target);

        // 转换操作
        operator uint32_t();
        operator int32_t();

        void begin(uint32_t index = 0);
        void end();

        bool ready() const;

        template<class T>
        void value(T& value) const;

    protected:
        void bind() override;
        void release() override;

        bool started_{false};
        GLenum target_;
        GLuint index_{0};
    };

    template<class T>
    void GlQuery::value(T& value) const{
        GLint params;
        glGetQueryObjectiv(id_, GL_QUERY_RESULT, &params);

        return T(params);
    }
}

#endif //LIV_OPENGL_GLQUERY_H
