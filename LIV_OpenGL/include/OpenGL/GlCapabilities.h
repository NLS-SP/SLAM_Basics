//
// Created by Robotics_qi on 2020/6/29.
//

#ifndef LIV_OPENGL_GLCAPABILITIES_H
#define LIV_OPENGL_GLCAPABILITIES_H

#include <memory>
#include <OpenGL/GlState.h>

namespace LIV_OpenGL{
    class GlCapabilities{
    public:
        static GlCapabilities& getInstance();

        //!@brief 得到参数的名字
        template<typename T>
        T get(GLenum variable) const;

        friend std::ostream& operator<<(std::ostream& out, const GlCapabilities& cap);

    protected:
        GlCapabilities();

        std::string stringify_name(GLenum name) const;
        std::string stringify_value(const std::pair<GLenum, GlState::GlStateVariable>& entry) const;

        template<typename T>
        void initGlParameter(GLenum name, uint32_t num_values);

        static std::unique_ptr<GlCapabilities> instance_;
        std::map<GLenum, GlState::GlStateVariable> state_;
    };

    template<typename T>
    T GlCapabilities::get(GLenum variable) const{
        if(state_.find(variable) == state_.end()) throw std::runtime_error("No such variable found in GlState.");
        auto it = state_.find(variable);

        if(it->second.type == GlState::GlStateVariable::INT){
            return T(it->second.vali[0]);
        }else if(it->second.type == GlState::GlStateVariable::FLOAT){
            return T(it->second.valf[0]);
        }else if(it->second.type == GlState::GlStateVariable::BOOL){
            return T(it->second.valb);
        }

        throw std::runtime_error("Unknown state variable type.");
    }
}

#endif //LIV_OPENGL_GLCAPABILITIES_H
