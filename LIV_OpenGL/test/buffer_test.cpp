//
// Created by Robotics_qi on 2020/6/22.
//

#include <gtest/gtest.h>
#include <OpenGL/GlBuffer.h>
#include <Eigen/Dense>
#include <random>
#include <types/test_utils.h>

using namespace LIV_OpenGL;

namespace {
    TEST(BufferTest, initTest){
        GlBuffer<float> buffer(BUFFER_TARGET::ARRAY_BUFFER, BUFFER_USAGE::DYNAMIC_DRAW);
        // 只有当内存被绑定的时候才可作为一种缓存区来对待
        buffer.bind();
        buffer.release();
        ASSERT_TRUE(glIsBuffer(buffer.id()));
    }

    TEST(BufferTest, assignTest) {
        uint32_t num_values = 17;
        std::vector<float> values(num_values);
        Random rand(1234);
        for (uint32_t i = 0; i < values.size(); ++i) values[i] = rand.getFloat();
        GlBuffer<float> buffer(BUFFER_TARGET::ARRAY_BUFFER, BUFFER_USAGE::DYNAMIC_DRAW);
        ASSERT_EQ(BUFFER_TARGET::ARRAY_BUFFER, buffer.target());
        ASSERT_EQ(BUFFER_USAGE::DYNAMIC_DRAW, buffer.usage());
        ASSERT_EQ(static_cast<size_t>(0), buffer.size());           // empty buffer
        ASSERT_EQ(static_cast<size_t>(0), buffer.capacity());       // real empty buffer.

        buffer.assign(values);
        ASSERT_EQ(values.size(), buffer.size());        // Now, not empty buffer.

        std::vector<float> buf;
        buffer.get(buf);
        ASSERT_EQ(values.size(), buf.size());

        for(uint32_t i = 0; i < values.size(); ++i) ASSERT_FLOAT_EQ(values[i], buf[i]);

        GlBuffer<float> assign_buffer(BUFFER_TARGET::ARRAY_BUFFER, BUFFER_USAGE::DYNAMIC_DRAW);
        assign_buffer.assign(buffer);

        buf.clear();
        assign_buffer.get(buf);
        ASSERT_EQ(values.size(), buf.size());

        for(uint32_t i = 0; i < values.size(); ++i) ASSERT_FLOAT_EQ(values[i], buf[i]);

        ASSERT_NO_THROW(CheckGLError());
    }

    TEST(BufferTest, reserveTest){
        GlBuffer<float> buffer(BUFFER_TARGET::ARRAY_BUFFER, BUFFER_USAGE::DYNAMIC_DRAW);

        buffer.reserve(1000);
        ASSERT_EQ(static_cast<size_t>(1000), buffer.capacity());
        ASSERT_EQ(static_cast<size_t>(0), buffer.size());
    }

    TEST(BufferTest, resizeTest){
        GlBuffer<float> buffer(BUFFER_TARGET::ARRAY_BUFFER, BUFFER_USAGE::DYNAMIC_DRAW);

        buffer.reserve(1000);
        ASSERT_EQ(static_cast<size_t>(1000), buffer.capacity());

        buffer.resize(123);
        ASSERT_EQ(static_cast<size_t>(123), buffer.size());
        ASSERT_EQ(static_cast<size_t>(1000), buffer.capacity());

        GlBuffer<float> buffer2(BUFFER_TARGET::ARRAY_BUFFER, BUFFER_USAGE::DYNAMIC_DRAW);
        buffer2.resize(145);
        ASSERT_EQ(static_cast<size_t>(145), buffer2.size());
        ASSERT_EQ(static_cast<size_t>(145), buffer2.capacity());

        ASSERT_NO_THROW(CheckGLError());
    }

    TEST(BufferTest, replaceTest){
        GlBuffer<int32_t> buffer(BUFFER_TARGET::ARRAY_BUFFER, BUFFER_USAGE::STATIC_DRAW);
        uint32_t num_values = 100;
        std::vector<int32_t> values(num_values);

        Random rand(1234);
        for(uint32_t i = 0; i < values.size(); ++i) values[i] = rand.getInt(100, 147);

        buffer.assign(values);
        ASSERT_EQ(values.size(), buffer.size());

        std::vector<int32_t> buf_values;
        buffer.get(buf_values);
        ASSERT_EQ(values.size(), buf_values.size());

        for(uint32_t i = 0; i < buf_values.size(); ++i) ASSERT_EQ(values[i], buf_values[i]);

        ASSERT_NO_THROW(CheckGLError());

        std::vector<int32_t> replace_values(13, 1);

        uint32_t offset = 10;
        buffer.replace(offset, replace_values);
        ASSERT_EQ(values.size(), buffer.size());

        buffer.get(buf_values);
        for(uint32_t i = 0; i < buf_values.size(); ++i){
            if(i < offset || i >= offset + replace_values.size()) ASSERT_EQ(values[i], buf_values[i]);
            else{
                ASSERT_EQ(replace_values[i - offset], buf_values[i]);
                values[i] = buf_values[i];
            }
        }

        ASSERT_NO_THROW(CheckGLError());
    }

    TEST(BufferTest, eigenTest) {
        GlBuffer<Eigen::Matrix4f> mats(BUFFER_TARGET::ARRAY_BUFFER, BUFFER_USAGE::STATIC_READ);
        std::vector<Eigen::Matrix4f> orig_mats;
        Eigen::Matrix4f t1;
        t1 << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;

        orig_mats.push_back(t1);
        Eigen::Matrix4f t2 = -1.0f * t1;
        orig_mats.push_back(t2);

        mats.assign(orig_mats);
        std::vector<Eigen::Matrix4f> buffered_mats;
        mats.get(buffered_mats);

        ASSERT_EQ(orig_mats.size(), buffered_mats.size());
        for (uint32_t k = 0; k < buffered_mats.size(); ++k) {
            const Eigen::Matrix4f &A = buffered_mats[k];
            const Eigen::Matrix4f &B = orig_mats[k];
            for (uint32_t i = 0; i < 4; ++i)
                for (uint32_t j = 0; j < 4; ++j)
                    ASSERT_LT(std::abs(A(i, j) - B(i, j)), 0.001f);
        }
    }
}