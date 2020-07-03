//
// Created by Robotics_qi on 2020/6/28.
//

#ifndef LIV_OPENGL_GLUTIL_H
#define LIV_OPENGL_GLUTIL_H

#include <cmath>
#include <Eigen/Dense>
#include <ostream>

//!@brief 一些基础类和方法定义
namespace LIV_OpenGL{
    //!@brief 二维向量【空间点表示？】
    struct vec2{
    public:
        vec2(): x(0.0f), y(0.0f){}
        vec2(float xx, float yy):x(xx),y(yy){}
        float x, y;
    };

    //!@brief 三维向量【三维空间表示】
    struct vec3{
    public:
        vec3():x(0.0f), y(0.0f), z(0.0f){}
        vec3(float xx, float yy, float zz):x(xx), y(yy), z(zz){}
        float x, y, z;
    };

    //!@brief 四维向量【齐次坐标表示】
    struct vec4{
    public:
        vec4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f){}
        float x, y, z, w;
    };

    //!@brief 在x,y,z方向上的平移
    Eigen::Matrix4f glTranslate(float x, float y, float z);
    Eigen::Matrix4f glScale(float x, float y, float z);

    //!@brief 沿着x轴方向进行旋转
    Eigen::Matrix4f glRotateX(float angle);

    //!@brief 沿着y轴方向进行旋转
    Eigen::Matrix4f glRotateY(float angle);

    //!@brief 沿着z轴方向进行旋转
    Eigen::Matrix4f glRotateZ(float angle);

    //!@brief 透视投影矩阵
    Eigen::Matrix4f glPerspective(float fov, float aspect, float zNear, float zFar);

    //!@brief 正视投影
    Eigen::Matrix4f glOrthographic(float left, float right, float bottom, float top, float zNear, float zFar);

    //!@brief 沿着（x,y,z）轴来进行一定角度的旋转
    Eigen::Matrix4f glRotateAxis(float angle, float x, float y, float z);

    //!@brief 将角度转换成弧度
    inline float radians(float deg) { return deg * M_PI / 180.0f; }

    //!@brief 将弧度转换成角度
    inline float degrees(float rad) { return rad * 180.0f / M_PI; }

    //!@brief 坐标系转换【Rose->OpenGL】
    struct RoSe2GL{
    public:
        // TODO 默认构造函数删除的意思是？
        RoSe2GL() = delete;
        static Eigen::Matrix4f matrix;
    };

    //!@brief 坐标系转换【OpenGL->Rose】
    struct GL2RoSe{
    public:
        GL2RoSe() = delete;

        static Eigen::Matrix4f matrix;
    };

    //!@brief 将【r,g,b】三原色编码成一个单点float型
    float rgb2float(float r, float g, float b);

    std::string extension(const std::string& path, int32_t level = 1);
}

//TODO Figure out whtat does std::ostream do
std::ostream& operator<<(std::ostream& stream, LIV_OpenGL::vec2& vec);
std::ostream& operator<<(std::ostream& stream, LIV_OpenGL::vec3& vec);
std::ostream& operator<<(std::ostream& stream, LIV_OpenGL::vec4& vec);

std::ostream& operator<<(std::ostream& stream, const LIV_OpenGL::vec2& vec);
std::ostream& operator<<(std::ostream& stream, const LIV_OpenGL::vec3& vec);
std::ostream& operator<<(std::ostream& stream, const LIV_OpenGL::vec4& vec);

#endif //LIV_OPENGL_GLUTIL_H
