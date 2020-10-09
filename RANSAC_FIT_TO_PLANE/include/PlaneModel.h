//
// Created by Robotics_qi on 2020/10/8.
//

#ifndef RANSAC_FIT_TO_PLANE_PLANEMODEL_H
#define RANSAC_FIT_TO_PLANE_PLANEMODEL_H

#include <Geometry.h>
#include <vector>

class PlaneModel{
public:
    PlaneModel(){}

    void initialize(std::vector<Point3D>& data);
public:
    float plane_a, plane_b, plane_c, plane_d;
    float plane_denominator;
};

#endif //RANSAC_FIT_TO_PLANE_PLANEMODEL_H
