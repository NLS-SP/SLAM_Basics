//
// Created by Robotics_qi on 2020/7/3.
//

#ifndef LIV_OPENGL_GLCOLOR_H
#define LIV_OPENGL_GLCOLOR_H
#include <cassert>
#include <stdint.h>
#include <OpenGL/GlUtil.h>

namespace LIV_OpenGL{
    //!@brief OpenGL的颜色对象，用[0, 1]区间来代表OpenGL中的颜色值
    class GlColor{
    public:
        GlColor() : R(0.0f), G(0.0f), B(0.0f), A(1.0f) {}

        GlColor(int32_t r, int32_t g, int32_t b, int32_t a = 255):
                R(float(r) / 255.), G(float(g) / 255.), B(float(b) / 255.), A(float(a) / 255.){}

        GlColor(float r, float g, float b, float a = 1.0f) : R(r), G(g), B(b), A(a){}

        //!@brief 转换成float
        operator float*(){ return &R;}

        // 奇怪，不是float的么？怎么变成数组的？
        operator const float*() const{ return &R; }

        inline float operator[](int i){return (&R)[i]; }

        float toFloat(){
            int32_t rgb = int32_t(round(R * 255.0f));
            rgb = (rgb << 8) + int32_t(round(G * 255.0f));
            rgb = (rgb << 8) + int32_t(round(B * 255.0f));

            return float(rgb);
        }

        static GlColor FromHSV(float h, float s, float v, float a = 1.0f);

        static GlColor FromRGB(uint8_t r,uint8_t g, uint8_t  b, uint8_t a = 255);

        void lighter(float factor = 1.0);
        void darker(float factor = 1.0);
        float R, G, B, A;

        //一些常见的颜色可以预先定义
        static const GlColor WHITE;
        static const GlColor GRAY;
        static const GlColor BLACK;
        static const GlColor RED;
        static const GlColor GREEN;
        static const GlColor BLUE;
        static const GlColor YELLOW;
        static const GlColor PINK;
        static const GlColor ORANGE;
        static const GlColor CYAN;
        static const GlColor GOLD;

    protected:

        void toHSV(float& h, float& s, float& v, float& a);
        void setHSV(float h, float s, float v, float a);
    };
}

#endif //LIV_OPENGL_GLCOLOR_H
