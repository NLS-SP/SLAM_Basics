//
// Created by Robotics_qi on 2020/7/2.
//
#include <gtest/gtest.h>
#include <OpenGL/GlState.h>
#include <OpenGL/GlTexture.h>
#include <OpenGL/GlFramebuffer.h>
#include <OpenGL/GlRenderbuffer.h>
#include <OpenGL/GlTextureRectangle.h>

using namespace LIV_OpenGL;

namespace{
    TEST(FramebufferTest, initTest){
        GlFramebuffer buf(10, 10);
        GlRenderbuffer rbo(10, 10, RENDERBUFFER_FORMAT::DEPTH_STENCIL);
        buf.attach(FRAMEBUFFER_ATTACHMENT::DEPTH_STENCIL, rbo);
        buf.bind();
        buf.release();

        ASSERT_TRUE(glIsFramebuffer(buf.id()));
        ASSERT_NO_THROW(CheckGLError());
    }

    TEST(FRAMEBUFFERTEST, testStatelessness){
        GlState priorState = GlState::queryAll();
        CheckGLError();

        GlFramebuffer buf(640, 480);
        if(priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
        ASSERT_EQ(true, (priorState == GlState::queryAll()));

        GlTexture texture(640, 480, TEXTURE_FORMAT::RGBA);
        GlRenderbuffer rbo(640, 480, RENDERBUFFER_FORMAT::DEPTH_STENCIL);
        if(priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
        ASSERT_EQ(true, (priorState == GlState::queryAll()));

        buf.attach(FRAMEBUFFER_ATTACHMENT::COLOR0, texture);
        if(priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
        ASSERT_EQ(true, (priorState == GlState::queryAll()));

        buf.attach(FRAMEBUFFER_ATTACHMENT::DEPTH_STENCIL, rbo);
        if(priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
        ASSERT_EQ(true, (priorState == GlState::queryAll()));

        buf.bind();

        buf.release();

        if(priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
        ASSERT_EQ(true, (priorState == GlState::queryAll()));

        buf.valid();
        if(priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
        ASSERT_EQ(true, (priorState == GlState::queryAll()));
    }
}