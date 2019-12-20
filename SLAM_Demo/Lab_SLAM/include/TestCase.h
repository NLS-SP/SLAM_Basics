//
// Created by gatsby on 2019-02-27.
//

#ifndef LAB_SLAM_TESTCASE_H
#define LAB_SLAM_TESTCASE_H

namespace Lab_SLAM{
    class testCase{
    public:

        void generate2D3D();
        void showTestCase();
        void setPointsBoundaries(float x_ini, float x_end, float y_ini, float y_end,
                                 float z_ini, float z_end);
        void setPixelBoundaries(ushort width_pixel, ushort height_pixel);
    private:
        // The 3D points boundary;
        float x_ini_, x_end_;
        float y_ini_, y_end_;
        float z_ini_, z_end_;

        // The 2D pixel boundaries
        ushort width_pixel_;
        ushort height_pixel_;
    };
}

#endif //LAB_SLAM_TESTCASE_H
