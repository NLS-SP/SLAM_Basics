//
// Created by Robotics_qi on 2020/7/2.
//


#include <gtest/gtest.h>
#include <OpenGL/GlState.h>
#include <OpenGL/GlTexture.h>
#include <OpenGL/GlTextureRectangle.h>

using namespace LIV_OpenGL;

TEST(TextureTest, loadTexture){}

TEST(TextureTest, assignTest){
    // 声明texture对象，此时还没有进行纹理区域绑定
    GlTexture texture(100, 50, TEXTURE_FORMAT::RGBA_FLOAT);
    ASSERT_NO_THROW(CheckGLError());
    std::vector<float> img(100 * 50 * 4);
    ASSERT_EQ(static_cast<size_t>(100 * 50 * 4), img.size());
    for(uint32_t i = 0; i < img.size(); ++i)
        img[i] = 1.45f * i;
    // 对纹理区域进行内存空间分配，但依然没有内容，说明Boundtexture_还没有绑定上
    texture.assign(PIXEL_FORMAT::RGBA, PIXEL_TYPE::FLOAT, &img[0]);
    ASSERT_NO_THROW(CheckGLError());

    std::vector<float> device_mem;
    texture.download(device_mem);
    ASSERT_EQ(img.size(), device_mem.size());
    for(uint32_t i = 0; i < img.size(); ++i) ASSERT_EQ(img[i], device_mem[i]);
}

TEST(TextureTest, copyTest){
    GlTexture texture(100, 50, TEXTURE_FORMAT::RGBA_FLOAT);
    ASSERT_NO_THROW(CheckGLError());

    // 假设创建一张img纹理图像，100*50的RGBA图像
    std::vector<float> img(100 * 50 * 4);
    for(uint32_t i = 0; i < img.size(); ++i)
        img[i] = 1.45f * i;

    // 这里应该是基于img来进行内存空间分配
    texture.assign(PIXEL_FORMAT::RGBA, PIXEL_TYPE::FLOAT, &img[0]);
    ASSERT_NO_THROW(CheckGLError());

    GlTexture texture2(100, 50, TEXTURE_FORMAT::RGBA_FLOAT);

    GlState state_before = GlState::queryAll();

    // 将texture中的内容拷贝到texture2中
    // 这里后面要好好阅读下，之前写的有问题
    texture2.copy(texture);

    GlState state_aftwards = GlState::queryAll();
    if(state_before != state_aftwards)
        state_before.difference(state_aftwards);

    ASSERT_TRUE(state_before == state_aftwards);

    std::vector<float> device_mem;
    // 总算弄清了，这里的download实际上是将texture内容从GPU中拷贝下来到CPU
    texture2.download(device_mem);
    ASSERT_EQ(img.size(), device_mem.size());

    for(uint32_t i = 0; i < img.size(); ++i)
        ASSERT_EQ(img[i], device_mem[i]);
}

// 突然想到是不是一个是深拷贝，一个是浅拷贝？
TEST(TextureTest, cloneTest){
    GlTexture texture(100, 50, TEXTURE_FORMAT::RGBA_FLOAT);
    ASSERT_NO_THROW(CheckGLError());

    std::vector<float> img(100 * 50 * 4);
    for(unsigned i = 0; i < img.size(); ++i) img[i] = 1234.3 * i;

    texture.assign(PIXEL_FORMAT::RGBA, PIXEL_TYPE::FLOAT, &img[0]);
    ASSERT_NO_THROW(CheckGLError());

    GlTexture clone = texture.clone();

    ASSERT_EQ(texture.width(), clone.width());
    ASSERT_EQ(texture.height(), clone.height());

    std::vector<float> device_mem;
    clone.download(device_mem);
    ASSERT_EQ(img.size(), device_mem.size());

    for(uint32_t i = 0; i < img.size(); ++i) ASSERT_EQ(img[i], device_mem[i]);
}

TEST(TextureRectangleTest, loadTexture){}

TEST(TextureRectangleTest, assignTest){
    GlTextureRectangle texture(100, 50, TEXTURE_FORMAT::RGBA_FLOAT);
    ASSERT_NO_THROW(CheckGLError());
    std::vector<float> img(100 * 50 * 4);
    ASSERT_EQ(static_cast<size_t>(100 * 50 * 4), img.size());
    for (uint32_t i = 0; i < img.size(); ++i) {
        img[i] = 1.45f * i;
    }
    texture.assign(PIXEL_FORMAT::RGBA, PIXEL_TYPE::FLOAT, &img[0]);
    ASSERT_NO_THROW(CheckGLError());

    std::vector<float> device_mem;
    texture.download(device_mem);
    ASSERT_EQ(img.size(), device_mem.size());
    for (uint32_t i = 0; i < img.size(); ++i) {
        ASSERT_EQ(img[i], device_mem[i]);
    }
}

TEST(TextureRectangleTest, copyTest){
    GlTextureRectangle texture(100, 50, TEXTURE_FORMAT::RGBA_FLOAT);
    ASSERT_NO_THROW(CheckGLError());

    std::vector<float> img(100 * 50 * 4);
    for(uint32_t i = 0; i < img.size(); ++i) img[i] = 1.45f * i;

    texture.assign(PIXEL_FORMAT::RGBA, PIXEL_TYPE::FLOAT, &img[0]);
    ASSERT_NO_THROW(CheckGLError());

    GlTextureRectangle texture2(100, 50, TEXTURE_FORMAT::RGBA_FLOAT);
    // 一样的，在这里出问题了，后面要好好核对下
    texture2.copy(texture);

    std::vector<float> device_mem;
    texture2.download(device_mem);

    ASSERT_EQ(img.size(), device_mem.size());

    for(uint32_t i = 0; i < img.size(); ++i) ASSERT_EQ(img[i], device_mem[i]);
}

TEST(TextureRectangleTest, cloneTest){
    GlTextureRectangle texture(100, 50, TEXTURE_FORMAT::RGBA_FLOAT);
    ASSERT_NO_THROW(CheckGLError());

    std::vector<float> img(100 * 50 * 4);
    for(uint32_t i = 0; i < img.size(); ++i) img[i] = 1234.3 * i;
    texture.assign(PIXEL_FORMAT::RGBA, PIXEL_TYPE::FLOAT, &img[0]);
    ASSERT_NO_THROW(CheckGLError());

    GlTextureRectangle clone = texture.clone();
    ASSERT_EQ(texture.width(), clone.width());
    ASSERT_EQ(texture.height(), clone.height());

    std::vector<float> device_mem;
    clone.download(device_mem);
    ASSERT_EQ(img.size(), device_mem.size());

    for(uint32_t i = 0; i < img.size(); ++i)
        ASSERT_EQ(img[i], device_mem[i]);
}