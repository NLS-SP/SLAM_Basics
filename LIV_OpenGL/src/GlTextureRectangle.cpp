//
// Created by Robotics_qi on 2020/6/30.
//

#include <vector>
#include <fstream>
#include <cassert>
#include <stdint.h>

#include <OpenGL/GlUtil.h>
#include <OpenGL/GlVertexArray.h>
#include <OpenGL/GlTextureRectangle.h>
#include <OpenGL/GlTexture.h>
#include <OpenGL/GlState.h>
#include <OpenGL/GlSampler.h>
#include <OpenGL/GlProgram.h>
#include <OpenGL/GlFramebuffer.h>

#include <boost/filesystem.hpp>

namespace LIV_OpenGL{
    GLuint GlTextureRectangle::boundTexture_ = 0;

    template <>
    void GlTextureRectangle::download<float>(std::vector<float>& data) const {
        data.resize(numComponents(format_) * width_ * height_);
        GLuint id = bindTransparently();

        GLenum pixFormat = GL_RGBA;

        switch (format_) {
            case TEXTURE_FORMAT::R:
            case TEXTURE_FORMAT::R_FLOAT:
                pixFormat = GL_RED;
                break;

            case TEXTURE_FORMAT::R_INTEGER:
                pixFormat = GL_RED_INTEGER;
                break;

            case TEXTURE_FORMAT::RG:
            case TEXTURE_FORMAT::RG_FLOAT:
                pixFormat = GL_RG;
                break;

            case TEXTURE_FORMAT::RG_INTEGER:
                pixFormat = GL_RG_INTEGER;
                break;

            case TEXTURE_FORMAT::RGB:
            case TEXTURE_FORMAT::RGB_FLOAT:
                pixFormat = GL_RGB;
                break;

            case TEXTURE_FORMAT::RGB_INTEGER:
                pixFormat = GL_RGB_INTEGER;
                break;

            case TEXTURE_FORMAT::RGBA:
            case TEXTURE_FORMAT::RGBA_FLOAT:
                pixFormat = GL_RGBA;
                break;

            case TEXTURE_FORMAT::RGBA_INTEGER:
                pixFormat = GL_RGBA_INTEGER;
                break;

            case TEXTURE_FORMAT::DEPTH:
                pixFormat = GL_DEPTH;
                break;

            case TEXTURE_FORMAT::DEPTH_STENCIL:
                pixFormat = GL_DEPTH_STENCIL;
                break;
        }

        glGetTexImage(GL_TEXTURE_RECTANGLE, 0, pixFormat, GL_FLOAT, reinterpret_cast<GLvoid*>(&data[0]));
        CheckGLError();
        releaseTransparently(id);
    }

    template<>
    void GlTextureRectangle::download<vec4>(std::vector<vec4>& data) const{
        data.resize(width_ * height_);
        GLuint id = bindTransparently();
        if(format_ == TEXTURE_FORMAT::R_INTEGER){
            std::vector<int32_t> d(width_ * height_);
            glGetTexImage(GL_TEXTURE_RECTANGLE, 0, GL_RED_INTEGER, GL_INT, reinterpret_cast<GLvoid*>(&d[0]));
            for(uint32_t i = 0; i < width_ * height_; ++i){
                data[i].x = d[i];
                data[i].y = 0;
                data[i].z = 0;
                data[i].w = 0;
            }
        }else{
            glGetTexImage(GL_TEXTURE_RECTANGLE, 0, GL_RGBA, GL_FLOAT, reinterpret_cast<GLvoid*>(&data[0]));
        }
        CheckGLError();
        releaseTransparently(id);
    }

    template<>
    void GlTextureRectangle::download(Eigen::Vector4f* ptr) const {
        GLuint id = bindTransparently();
        glGetTexImage(GL_TEXTURE_RECTANGLE, 0, GL_RGBA, GL_FLOAT, reinterpret_cast<GLvoid*>(ptr));
        releaseTransparently(id);

        CheckGLError();
    }

    template<>
    void GlTextureRectangle::download(PIXEL_FORMAT pixelfmt, float* ptr) const {
        GLuint id = bindTransparently();
        glGetTexImage(GL_TEXTURE_RECTANGLE, 0, static_cast<GLenum>(pixelfmt), GL_FLOAT, reinterpret_cast<GLvoid*>(ptr));
        releaseTransparently(id);

        CheckGLError();
    }

    template<>
    void GlTextureRectangle::download(PIXEL_FORMAT pixelfmt, int32_t* ptr) const{
        GLuint id = bindTransparently();
        glGetTexImage(GL_TEXTURE_RECTANGLE, 0, static_cast<GLenum>(pixelfmt), GL_INT, reinterpret_cast<GLvoid*>(ptr));
        releaseTransparently(id);

        CheckGLError();
    }

    GlTextureRectangle::GlTextureRectangle(uint32_t width, uint32_t height, TEXTURE_FORMAT format)
                        :width_(width), height_(height), format_(format){
        glGenTextures(1, &id_);
        ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr){
            glDeleteTextures(1, ptr);
            delete ptr;
        });

        // allocate space.
        GLuint old_id = bindTransparently();
        allocateMemory();
        CheckGLError();

        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);


        releaseTransparently(old_id);

        CheckGLError();
    }

    GlTextureRectangle::~GlTextureRectangle() {
        if(boundTexture_ == id_){
            release();
            assert((boundTexture_ != id_) && "Texture object still bound.");
        }
    }

    void GlTextureRectangle::copy(const GlTextureRectangle& other){
        GLboolean depthTest;
        GLint ov[4];
        GLint old_fbo;

        glGetIntegerv(GL_VIEWPORT, ov);
        glGetBooleanv(GL_DEPTH_TEST, &depthTest);
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &old_fbo);

        GlVertexArray vao;

        // Important: Sampler is needed to complete texture specification! Despite that the texture defines
        // or should define all the needed stuff.
        GlSampler sampler;
        sampler.setMinifyingOperation(TEX_MIN_OP::NEAREST);
        sampler.setMagnifyingOperation(TEX_MAG_OP::NEAREST);
        // Note: For GL_TEXTURE_RECTANLGE GL_REPEAT is not an option.
        // FIXME: Somehow avoid setting of wrong wrapping options!
        sampler.setWrapOperation(TEX_WRAP_OP::CLAMP_TO_BORDER, TEX_WRAP_OP::CLAMP_TO_BORDER);

        // create a framebuffer with destination texture and use fragment shader for copy.
        GlFramebuffer copy_fbo(width_, height_, FRAMEBUFFER_TARGET ::DRAW);
        GlRenderbuffer rbo(width_, height_, RENDERBUFFER_FORMAT::DEPTH_STENCIL);
        copy_fbo.attach(FRAMEBUFFER_ATTACHMENT::COLOR0, *this);
        copy_fbo.attach(FRAMEBUFFER_ATTACHMENT::DEPTH_STENCIL, rbo);
        assert(copy_fbo.valid());

        std::string empty_vert = "#version 330 core\nvoid main(){}";
        std::string quad_geom = "#version 330 core\nlayout(points) in;\nlayout(triangle_strip, max_vertices = 4) out;\n";
        quad_geom += "out vec2 texCoords;\nvoid main(){\n";
        quad_geom += "  gl_Position = vec4(1.0, 1.0, 0.0, 1.0);\ntexCoords = vec2(1.0, 1.0);\nEmitVertex();\n";
        quad_geom += "  gl_Position = vec4(-1.0, 1.0, 0.0, 1.0);\ntexCoords = vec2(0.0, 1.0);\nEmitVertex();\n";
        quad_geom += "  gl_Position = vec4(1.0,-1.0, 0.0, 1.0);\ntexCoords = vec2(1.0, 0.0);\nEmitVertex();\n";
        quad_geom += "  gl_Position = vec4(-1.0,-1.0, 0.0, 1.0);\ntexCoords = vec2(0.0, 0.0);\nEmitVertex();\n";
        quad_geom += "  EndPrimitive();\n}";
        std::string copy_frag = "#version 330 core\nin vec2 texCoords; \n";

        switch (other.format_) {
            case TEXTURE_FORMAT::R_INTEGER:
            case TEXTURE_FORMAT::RG_INTEGER:
            case TEXTURE_FORMAT::RGB_INTEGER:
            case TEXTURE_FORMAT::RGBA_INTEGER:
                copy_frag += "uniform isampler2DRect tex_other;\n";
                break;
            default:
                copy_frag += "uniform sampler2DRect tex_other;\n";
        }
        // do we need to consider input/ouptut formats?
        copy_frag += "out vec4 color;\nvoid main(){\n  color = texture(tex_other, texCoords * textureSize(tex_other)); \n}";

        GlProgram copy_program;
        copy_program.attach(GlShader(SHADER_TYPE::VERTEX_SHADER, empty_vert));
        copy_program.attach(GlShader(SHADER_TYPE::GEOMETRY_SHADER, quad_geom));
        copy_program.attach(GlShader(SHADER_TYPE::FRAGMENT_SHADER, copy_frag));
        copy_program.link();
        copy_program.setUniform(GlUniform<int32_t>("tex_other", 0));

        copy_fbo.bind();
        copy_program.bind();
        vao.bind();

        glDisable(GL_DEPTH_TEST);
        glViewport(0, 0, width_, height_);  // set viewport to new values

        glActiveTexture(GL_TEXTURE0);
        uint32_t old_id = other.bindTransparently();
        sampler.bind(0);

        glDrawArrays(GL_POINTS, 0, 1);

        sampler.release(0);
        vao.release();
        other.releaseTransparently(old_id);
        copy_program.release();

        glEnable(GL_DEPTH_TEST);
        copy_fbo.release();

        glFinish();

        // restore settings.
        glViewport(ov[0], ov[1], ov[2], ov[3]);
        if (depthTest)
            glEnable(GL_DEPTH_TEST);
        else
            glDisable(GL_DEPTH_TEST);

        // restore old framebuffer content.
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, old_fbo);
    }

    //!@brief 将其他纹理数据拷贝过来
    void GlTextureRectangle::copy(const GlTexture& other){
        GLboolean depthTest;
        GLint ov[4];
        GLint old_fbo;

        glGetIntegerv(GL_VIEWPORT, ov);
        glGetBooleanv(GL_DEPTH_TEST, &depthTest);
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &old_fbo);

        GlVertexArray vao;

        //Sampler对于纹理区域来说是必须的
        GlSampler sampler;
        sampler.setMinifyingOperation(TEX_MIN_OP::NEAREST);
        sampler.setMagnifyingOperation(TEX_MAG_OP::NEAREST);
        sampler.setWrapOperation(TEX_WRAP_OP::CLAMP_TO_BORDER, TEX_WRAP_OP::CLAMP_TO_BORDER);

        //对于目标区域创建需要的缓冲区
        GlFramebuffer copy_fbo(width_, height_, FRAMEBUFFER_TARGET::DRAW);
        GlRenderbuffer rbo(width_, height_, RENDERBUFFER_FORMAT::DEPTH_STENCIL);
        copy_fbo.attach(FRAMEBUFFER_ATTACHMENT::COLOR0, *this);
        copy_fbo.attach(FRAMEBUFFER_ATTACHMENT::DEPTH_STENCIL, rbo);
        assert(copy_fbo.valid());

        std::string empty_vert = "#version 330 core\n void main(){}";
        std::string quad_geom  = "#version 330 core\n layout(points) in;\nlayout(triangle_strip, max_vertices = 4) out;\n";
        quad_geom += "out vec2 texCoords;\nvoid main(){\n";
        quad_geom += "  gl_Position = vec(1.0, 1.0, 0.0, 1.0);\ntexCoords = vec2(1.0, 1.0);\nEmitVertex();\n";
        quad_geom += "  gl_Position = vec(-1.0, 1.0, 0.0, 1.0);\ntexCoords = vec2(0.0, 1.0);\nEmitVertex();\n";
        quad_geom += "  gl_Position = vec(1.0, -1.0, 0.0, 1.0);\ntexCoords = vec2(1.0, 0.0);\nEmitVertex();\n";
        quad_geom += "  gl_Position = vec(-1.0, -1.0, 0.0, 1.0);\ntexCoords = vec2(0.0, 0.0);\nEmitVertex();\n";
        quad_geom += "  EndPrimitive();\n}";
        std::string copy_frag = "#version 330 core\nin vec2 texCoords;\n";

        switch(other.format_){
            case TEXTURE_FORMAT::R_INTEGER:
            case TEXTURE_FORMAT::RG_INTEGER:
            case TEXTURE_FORMAT::RGB_INTEGER:
            case TEXTURE_FORMAT::RGBA_INTEGER:
                copy_frag += "uniform isampler2D tex_other;\n";
                break;
            default:
                copy_frag += "uniform sampler2D tex_other;\n";
        }

        //我们是否需要考虑输入输出的格式
        copy_frag += "out vec4 color;\nvoid main(){\n color = texture(tex_other, texCoords); \n}";

        GlProgram copy_program;
        copy_program.attach(GlShader(SHADER_TYPE::VERTEX_SHADER, empty_vert));
        copy_program.attach(GlShader(SHADER_TYPE::GEOMETRY_SHADER, quad_geom));
        copy_program.attach(GlShader(SHADER_TYPE::FRAGMENT_SHADER, copy_frag));
        copy_program.link();
        copy_program.setUniform(GlUniform<int32_t>("tex_other", 0));

        copy_fbo.bind();
        copy_program.bind();
        vao.bind();

        glDisable(GL_DEPTH_TEST);
        glViewport(0, 0, width_, height_);

        glActiveTexture(GL_TEXTURE0);
        uint32_t old_id = other.bindTransparently();
        sampler.bind(0);

        glDrawArrays(GL_POINTS, 0, 1);

        sampler.release(0);
        vao.release();
        other.releaseTransparently(old_id);
        copy_program.release();

        glEnable(GL_DEPTH_TEST);
        copy_fbo.release();

        glFinish();

        //restore settings.
        glViewport(ov[0], ov[1], ov[2], ov[3]);
        if(depthTest)
            glEnable(GL_DEPTH_TEST);
        else
            glDisable(GL_DEPTH_TEST);

        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, old_fbo);
    }

    GlTextureRectangle GlTextureRectangle::clone() const{
        GlTextureRectangle tex(width_, height_, format_);
        tex.copy(*this);
        return tex;
    }

    void GlTextureRectangle::bind(){
        glBindTexture(GL_TEXTURE_RECTANGLE, id_);
        boundTexture_ = id_;
    }

    void GlTextureRectangle::release(){
        glBindTexture(GL_TEXTURE_RECTANGLE, 0);
        boundTexture_ = 0;
    }

    void GlTextureRectangle::setMinifyingOperation(LIV_OpenGL::TEX_RECT_MIN_OP minifyingOperation) {
        GLuint old_id = bindTransparently();
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, static_cast<GLenum>(minifyingOperation));
        releaseTransparently(old_id);
    }

    void GlTextureRectangle::setMagnifyingOperation(LIV_OpenGL::TEX_RECT_MAG_OP magnifyingOperation) {
        GLuint old_id = bindTransparently();
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, static_cast<GLenum>(magnifyingOperation));
        releaseTransparently(old_id);
    }

    void GlTextureRectangle::setWrapOperation(TEX_RECT_WRAP_OP wrap_s, TEX_RECT_WRAP_OP wrap_t){
        GLuint old_id = bindTransparently();
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, static_cast<GLenum>(wrap_s));
        glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, static_cast<GLenum>(wrap_t));
        releaseTransparently(old_id);
    }

    void GlTextureRectangle::setTextureSwizzle(TEX_RECT_SWIZZLE red, TEX_RECT_SWIZZLE green, TEX_RECT_SWIZZLE blue, TEX_RECT_SWIZZLE alpha){
        GLint swizzleMask[] = {static_cast<GLint>(red), static_cast<GLint>(green), static_cast<GLint>(blue), static_cast<GLint>(alpha)};
        GLuint old_id = bindTransparently();
        glTexParameteriv(GL_TEXTURE_RECTANGLE, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
        releaseTransparently(old_id);
    }

    bool GlTextureRectangle::save(const std::string& file_name) const{
        std::string ext = LIV_OpenGL::extension(file_name);
        if(ext == ".ppm"){
            std::ofstream out(file_name.c_str());
            out << "P6" << std::endl;
            out << width_ << std::endl;
            out << height_ << std::endl;
            out << "255" << std::endl;

            std::vector<float> colors(width_ * height_ * 3);
            GLuint id = bindTransparently();
            glGetTexImage(GL_TEXTURE_RECTANGLE, 0, GL_RGB, GL_FLOAT, reinterpret_cast<GLvoid*>(&colors[0]));
            CheckGLError();
            releaseTransparently(id);

            std::vector<char> data(width_ * height_ * 3);

            for(uint32_t x = 0; x < width_; ++x){
                for(uint32_t y = 0; y < height_; ++y){
                    int32_t r = 255 * colors[3 * (x + (height_ - 1 - y) * width_)];
                    int32_t g = 255 * colors[3 * (x + (height_ - 1 - y) * width_) + 1];
                    int32_t b = 255 * colors[3 * (x + (height_ - 1 - y) * width_) + 2];
                    data[3 * (x + y * width_)] = (char)std::max(std::min(255, r), 0);
                    data[3 * (x + y * width_) + 1] = (char)std::max(std::min(255, g), 0);
                    data[3 * (x + y * width_) + 2] = (char)std::max(std::min(255, b), 0);
                }
            }
            out.write(&data[0], width_ * height_ * 3);
            out.close();
            return true;
        }
        return false;
    }

    GlTextureRectangle GlTextureRectangle::loadTexture(const std::string& file_name){
        assert(false && "loading of textures not implemented");
        return GlTextureRectangle(0, 0, TEXTURE_FORMAT::RGBA);
    }

    GLuint GlTextureRectangle::bindTransparently() const {
        if(boundTexture_ == id_) return id_;
        glBindTexture(GL_TEXTURE_RECTANGLE, id_);
        return boundTexture_;
    }

    void GlTextureRectangle::releaseTransparently(GLuint old_id) const {
        if(old_id == id_) return;
        glBindTexture(GL_TEXTURE_RECTANGLE, old_id);
    }

    void GlTextureRectangle::resize(uint32_t width, uint32_t height){
        GLuint old_id = bindTransparently();
        allocateMemory();
        releaseTransparently(old_id);
        CheckGLError();
    }

    uint32_t GlTextureRectangle::width() const{
        return width_;
    }

    uint32_t GlTextureRectangle::height() const{
        return height_;
    }

    uint32_t GlTextureRectangle::numComponents(TEXTURE_FORMAT fmt) {
        switch(fmt) {
            case TEXTURE_FORMAT::R:
            case TEXTURE_FORMAT::R_INTEGER:
            case TEXTURE_FORMAT::R_FLOAT:
                return 1;
            case TEXTURE_FORMAT::RG:
            case TEXTURE_FORMAT::RG_INTEGER:
            case TEXTURE_FORMAT::RG_FLOAT:
                return 2;
            case TEXTURE_FORMAT::RGB:
            case TEXTURE_FORMAT::RGB_INTEGER:
            case TEXTURE_FORMAT::RGB_FLOAT:
                return 3;
            case TEXTURE_FORMAT::RGBA:
            case TEXTURE_FORMAT::RGBA_INTEGER:
            case TEXTURE_FORMAT::RGBA_FLOAT:
                return 4;
            case TEXTURE_FORMAT::DEPTH:
                return 1;
            case TEXTURE_FORMAT::DEPTH_STENCIL:
                return 2;
        }
    }

    void GlTextureRectangle::allocateMemory() {
        CheckGLError();
        if(width_ == 0 || height_ == 0) return;

        GLint texFormat = static_cast<GLint>(format_);
        GLenum pixFormat = GL_RGBA;
        GLenum pixType = GL_UNSIGNED_BYTE;

        switch(format_){
            case TEXTURE_FORMAT::R:
            case TEXTURE_FORMAT::RG:
            case TEXTURE_FORMAT::RGB:
            case TEXTURE_FORMAT::RGBA:
                pixType = GL_UNSIGNED_BYTE;
                break;

            case TEXTURE_FORMAT::R_INTEGER:
            case TEXTURE_FORMAT::RG_INTEGER:
            case TEXTURE_FORMAT::RGB_INTEGER:
            case TEXTURE_FORMAT::RGBA_INTEGER:
                pixType = GL_INT;
                break;

            case TEXTURE_FORMAT::R_FLOAT:
            case TEXTURE_FORMAT::RG_FLOAT:
            case TEXTURE_FORMAT::RGB_FLOAT:
            case TEXTURE_FORMAT::RGBA_FLOAT:
                pixType = GL_FLOAT;
                break;

            case TEXTURE_FORMAT::DEPTH:
                pixType = GL_UNSIGNED_BYTE;
                break;

            case TEXTURE_FORMAT::DEPTH_STENCIL:
                pixType = GL_DEPTH24_STENCIL8;
                break;
        }

        switch(format_){
            case TEXTURE_FORMAT::R:
            case TEXTURE_FORMAT::R_FLOAT:
                pixFormat = GL_RED;
                break;

            case TEXTURE_FORMAT::R_INTEGER:
                pixFormat = GL_RED_INTEGER;
                break;

            case TEXTURE_FORMAT::RG:
            case TEXTURE_FORMAT::RG_FLOAT:
                pixFormat = GL_RG;
                break;

            case TEXTURE_FORMAT::RG_INTEGER:
                pixFormat = GL_RG_INTEGER;
                break;

            case TEXTURE_FORMAT::RGB:
            case TEXTURE_FORMAT::RGB_FLOAT:
                pixFormat = GL_RGB;
                break;

            case TEXTURE_FORMAT::RGB_INTEGER:
                pixFormat = GL_RGB_INTEGER;
                break;

            case TEXTURE_FORMAT::RGBA:
            case TEXTURE_FORMAT::RGBA_FLOAT:
                pixFormat = GL_RGBA;
                break;

            case TEXTURE_FORMAT::RGBA_INTEGER:
                pixFormat = GL_RGBA_INTEGER;

            case TEXTURE_FORMAT::DEPTH:
                pixFormat = GL_DEPTH;
                break;

            case TEXTURE_FORMAT::DEPTH_STENCIL:
                pixFormat = GL_DEPTH_STENCIL;
                break;
        }
        glTexImage2D(GL_TEXTURE_RECTANGLE, 0, texFormat, width_, height_, 0, pixFormat, pixType, nullptr);
        CheckGLError();
    }
}