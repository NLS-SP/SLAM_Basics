//
// Created by Robotics_qi on 2020/6/22.
//

#ifndef LIV_OPENGL_GLOBJECT_H
#define LIV_OPENGL_GLOBJECT_H
#include <memory>
#include <OpenGL/GlBase.h>

namespace LIV_OpenGL{
    /** *
     * !@brief OpenGL中的对象模型（纯虚类）
     * 每个OpenGL中的对象应该包含
     *      一个识别器（确定在内存中的位置）
     *      一个智能指针（用于管理和释放OpenGL的资源）
     ** */
     class GlObject{
     public:
         virtual ~GlObject(){}

         //!@brief return the OpenGL's object identifer
         inline virtual GLuint id() const { return id_; }

         //!@brief bind/active the object.
         virtual void bind() = 0;

         //!@brief unbind/deactive the object.
         virtual void release() = 0;

     protected:
         GLuint id_{0};
         std::shared_ptr<GLuint> ptr_;
     };
}


#endif //LIV_OPENGL_GLOBJECT_H
