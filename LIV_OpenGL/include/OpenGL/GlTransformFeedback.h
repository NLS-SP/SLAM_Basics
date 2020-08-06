//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLTRANSFORMFEEDBACK_H
#define LIV_OPENGL_GLTRANSFORMFEEDBACK_H

#include <sstream>
#include <string>

#include <OpenGL/GlBuffer.h>
#include <OpenGL/GlCapabilities.h>
#include <OpenGL/GlObject.h>
#include <OpenGL/GlQuery.h>

namespace LIV_OpenGL{
    class GlProgram;

    //!@brief 用于转换传递的主要模块
    enum class TRANSFORM_FEEDBACK_MODE{
        POINTS = GL_POINTS,
        LINES = GL_LINES,
        TRIANGLES = GL_TRIANGLES
    };

    /***
     * !@brief OpenGL中的转换回调函数：在光栅化前将顶点和图元转换到缓冲区域中进行保存
     */
     class GlTransformFeedback : public GlObject{
     public:
         friend class GlProgram;

         GlTransformFeedback();

         //!@brief 将transform feedback和当前的画图绑定在一起
         void bind() override;

         //!@brief 对图元开始转换
         void begin(TRANSFORM_FEEDBACK_MODE mode);

         //!@brief 转换后的主元写入缓冲区中
         uint32_t end();

#if __GL_VERSION >= 400L
         void pause();

         void resume();

         void draw(GLenum mode) const;
#endif

         //!@brief 释放当前的转换函数
         void release() override;


         template<typename T>
         void attach(const std::vector<std::string>& varyings, GlBuffer<T>& buffer);

     protected:
         void registerVaryings(GLuint program_id);

         std::shared_ptr<bool> bound_;
         std::shared_ptr<bool> linked_;
         std::vector<std::pair<std::vector<std::string>, std::shared_ptr<GLuint>>> buffers_;
         GlQuery countquery_{QUERY_TARGET::TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN};
     };

     template<typename T>
     void GlTransformFeedback::attach(const std::vector<std::string> &varyings, LIV_OpenGL::GlBuffer<T> &buffer) {
         uint32_t maxBuffers = GlCapabilities::getInstance().get<int32_t>(GL_MAX_TRANSFORM_FEEDBACK_BUFFERS);
         if(buffers_.size() + 1 > maxBuffers){
             std::stringstream msg;
             msg << "Only " << maxBuffers << " transform feedback buffers allowed. See also GL_MAX_TRANSFORM_FEEDBACK_BUFFERS";
             throw std::runtime_error(msg.str());
         }
         buffers_.push_back(std::make_pair(varyings, buffer.ptr_));
     }
}

#endif //LIV_OPENGL_GLTRANSFORMFEEDBACK_H
