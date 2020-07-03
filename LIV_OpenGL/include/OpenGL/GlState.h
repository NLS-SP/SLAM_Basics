//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLSTATE_H
#define LIV_OPENGL_GLSTATE_H

#include <OpenGL/GlBase.h>
#include <cstdarg>
#include <ostream>
#include <map>

namespace LIV_OpenGL{
    //!@brief OpenGL对象，可以通过glGet来查询
    class GlState{
    public:
        class GlStateVariable{
        public:
            GlStateVariable(bool value);
            GlStateVariable(int32_t i1);
            GlStateVariable(int32_t i1, int32_t i2);
            GlStateVariable(int32_t i1, int32_t i2, int32_t i3);
            GlStateVariable(int32_t i1, int32_t i2, int32_t i3, int32_t i4);
            GlStateVariable(float f1);
            GlStateVariable(float f1, float f2);
            GlStateVariable(float f1, float f2, float f3);
            GlStateVariable(float f1, float f2, float f3, float f4);

            bool operator==(const GlStateVariable& other) const;
            bool operator!=(const GlStateVariable& other) const;

            enum DATA_TYPE{BOOL, INT, FLOAT};
            DATA_TYPE  type;
            union{
                bool    valb;
                int32_t vali[4];
                float   valf[4];
            };
            uint32_t size;
        };

        //!@brief 得到指定变量的状态
        template<typename T>
        T get(GLenum variable) const;

        //!@brief
        void restore();

        //!@brief 查询当前所有的OpenGL状态
        static GlState queryAll();

        //!@brief 对比状态并打印出出来状态的区别
        void difference(const GlState& other) const;

        //!@brief 对比是不是所有的状态都一样
        bool operator==(const GlState& other) const;

        bool operator!=(const GlState& other) const;

        //TODO: 明确ostream的作用
        friend std::ostream& operator<<(std::ostream& stream, const GlState& state);

    protected:
        //TODO 单例模式？
        GlState();

        std::string stringify_name(GLenum value) const;
        std::string stringify_value(const std::pair<GLenum, GlState::GlStateVariable>& entry) const;

        template<typename T>
        void initGlParameter(GLenum name, uint32_t num_values);

        std::map<GLenum, GlStateVariable> state_;
    };

    template<typename T>
    T GlState::get(GLenum variable) const{
        if(state_.find(variable) == state_.end()) throw std::runtime_error("No such variable found in GlState.");
        auto it = state_.find(variable);

        if(it->second.type == GlStateVariable::INT) return T(it->second.vali[0]);
        else if(it->second.type == GlStateVariable::FLOAT) return T(it->second.valf[0]);
        else if(it->second.type == GlStateVariable::BOOL) return T(it->second.valb);

        // 这里实际上应该永远不会运行到
        throw std::runtime_error("Unknown state variable type.");
    }
}

#endif //LIV_OPENGL_GLSTATE_H
