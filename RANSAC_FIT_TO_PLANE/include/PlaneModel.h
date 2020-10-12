//
// Created by Robotics_qi on 2020/10/8.
//

#ifndef RANSAC_FIT_TO_PLANE_PLANEMODEL_H
#define RANSAC_FIT_TO_PLANE_PLANEMODEL_H

#include <Geometry.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/SVD>

class PlaneModel{
public:
    PlaneModel(){}
    PlaneModel(double a, double b, double c, double d):
              plane_a(a), plane_b(b), plane_c(c), plane_d(d){
        plane_denominator = plane_a * plane_a + plane_b * plane_b + plane_c * plane_c + plane_d * plane_d;
    }

    void parameter_estimate(const std::vector<Point3D>& fitting_points);

public:
    float plane_a, plane_b, plane_c, plane_d;
    float plane_denominator;
};

#endif //RANSAC_FIT_TO_PLANE_PLANEMODEL_H

