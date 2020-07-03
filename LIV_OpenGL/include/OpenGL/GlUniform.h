//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLUNIFORM_H
#define LIV_OPENGL_GLUNIFORM_H

#include <OpenGL/GlBase.h>

namespace LIV_OpenGL{
    class GlProgram;

    class GlAbstractUniform{
    public:
        friend class GlProgram;

        GlAbstractUniform(const std::string& name) : name_(name) {}

        virtual ~GlAbstractUniform(){}

        //!@brief 得到当前单元的名称
        const std::string& name() const { return name_; }



    protected:

        virtual void bind(GLuint program_id) const = 0;

        std::string name_;
    };

    template<class T>
    class GlUniform : public GlAbstractUniform{
    public:
        friend class GlProgram;

        //!@brief 创建单元
        GlUniform(const std::string& name, const T& data);

        GlUniform<T>& operator=(const T& rhs);

        operator T();
        operator T() const;

        const T& value() const;

        void bind(GLuint program_id) const override;

        T data_;
    };

    template<class T>
    GlUniform<T>::GlUniform(const std::string &name, const T &data)
                : GlAbstractUniform(name), data_(data){
    }

    template<class T>
    GlUniform<T>& GlUniform<T>::operator=(const T& rhs){
        data_ = rhs;

        return *this;
    }

    template<class T>
    GlUniform<T>::operator T(){
        return data_;
    }

    template<class T>
    GlUniform<T>::operator T() const{
        return data_;
    }

    template<class T>
    const T& GlUniform<T>::value() const{
        return data_;
    }
}

#endif //LIV_OPENGL_GLUNIFORM_H
