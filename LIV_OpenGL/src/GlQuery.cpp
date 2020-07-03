//
// Created by Robotics_qi on 2020/6/29.
//

#include <OpenGL/GlQuery.h>
#include <OpenGL/GlException.h>


namespace LIV_OpenGL{
    template<>
    void GlQuery::value<int32_t>(int32_t& value) const{
        GLint params;
        glGetQueryObjectiv(id_, GL_QUERY_RESULT, &params);
        value = params;
    }
    template<>
    void GlQuery::value<uint32_t>(uint32_t& value) const{
        GLuint params;
        glGetQueryObjectuiv(id_, GL_QUERY_RESULT, &params);
        value = params;
    }

    GlQuery::GlQuery(QUERY_TARGET target) : target_(static_cast<GLenum>(target)){
        glGenQueries(1, &id_);
        ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr){
            glDeleteQueries(1, ptr);
            delete ptr;
        });
    }

    // 转换操作
    GlQuery::operator uint32_t(){
        GLuint params = 0;
        glGetQueryObjectuiv(id_, GL_QUERY_RESULT, &params);
        return params;
    }

    GlQuery::operator int32_t(){
        GLint params = 0;
        glGetQueryObjectiv(id_, GL_QUERY_RESULT, &params);
        return params;
    }

    void GlQuery::begin(uint32_t index){
        if(started_) throw GlQueryError("Query object already active.");

#if __GL_VERSION >= 400L
        if(target_ == GL_TIME_ELAPSED || target_ = GL_ANY_SAMPLES_PASSED || target_ = GL_SAMPLES_PASSED) index = 0;
        glBeginQueryIndexed(target_, index, id_);
        index_ = index;
#else
        glBeginQuery(target_, id_);
        index_ = 0;
#endif
        started_ = true;
    }

    void GlQuery::end(){
#if __GL_VERSION >= 400L
        glEndQueryIndexed(target_, index_);
#else
        glEndQuery(target_);
#endif
        started_ = false;
    }

    //!@brief 回溯查询结果是可查询的
    bool GlQuery::ready() const{
        GLint params;
        glGetQueryObjectiv(id_, GL_QUERY_RESULT_AVAILABLE, &params);

        return (params == GL_TRUE);
    }

    //!@brief 不知道这两个函数有什么用
    void GlQuery::bind(){}

    void GlQuery::release(){}
}