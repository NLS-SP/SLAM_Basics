//
// Created by gatsby on 2019-02-27.
//

/*
 * This program is built for generating the test data for different kinds of algorithms.
 * */

#include "Lab_SLAM/Common.h"
#include "Lab_SLAM/Camera.h"
#include "TestCase.h"
namespace Lab_SLAM{
    testCase::setPointsBoundaries(float x_ini, float x_end, float y_ini, float y_end,
            float z_ini, float z_end): x_ini_(x_ini), x_end_(x_end), y_ini_(y_ini),
            y_end_(y_end), z_ini_(z_ini), z_end_(z_end){}

    testCase::setPixelBoundaries(ushort width_pixel, ushort height_pixel): width_pixel_(width_pixel),
            height_pixel_(height_pixel){}

    testCase::generate2D3D();

    testCase::showTestCase();
}