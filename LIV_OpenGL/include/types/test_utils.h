//
// Created by Robotics_qi on 2020/6/23.
//

#ifndef LIV_OPENGL_TEST_UTILS_H
#define LIV_OPENGL_TEST_UTILS_H
#include <random>

namespace LIV_OpenGL{
    class Random{
    public:
        Random(): uniform_(0.0f, 1.0f), normal_(0.0f, 1.0f){}
        explicit Random(uint32_t seed) : rng_(seed), uniform_(0.0f, 1.0f), normal_(0.0, 1.0){}

        //!@brief 返回在[0, n-1]区间内的随机整数
        int32_t getInt(int32_t n) { return static_cast<int32_t>(n * uniform_(rng_)); }

        //!@brief 返回在[min, max]区间内的随机整数
        int32_t getInt(int32_t min, int32_t max){ return static_cast<int32_t>((max - min) * uniform_(rng_) + min);}

        //!@brief 这里是为了和STL一起使用，作用也是返回区间[0, n-1]中的任意整数
        int32_t operator()(int32_t n){ return getInt(n); }

        //!@brief 返回区间[0, 1]中的随机浮点数
        float getFloat(){ return static_cast<float>(uniform_(rng_)); }

        //!@brief 利用高斯随机分布(均值为0，方差为1)返回一个随机浮点数
        float getGaussianFloat(){ return static_cast<float>(uniform_(rng_)); }
        //!@brief 返回一个双精度浮点数
        float getDouble(){ return uniform_(rng_); }

        //!@brief 利用高斯随机分布(均值为0，方差为1)返回一个双精度随机浮点数
        double getGaussianDouble(){ return normal_(rng_); }

    protected:
        std::default_random_engine rng_;
        std::uniform_real_distribution<double> uniform_;
        std::normal_distribution<double> normal_;
    };
}

#endif //LIV_OPENGL_TEST_UTILS_H
