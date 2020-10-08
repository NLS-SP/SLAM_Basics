//
// Created by Robotics_qi on 2020/10/8.
//

#include <PlaneModel.h>


void PlaneModel::initialize(std::vector<Point3D> &data) {
    if(data.size() != 3)
        throw std::runtime_error("Plane3D Model- Number of input parameters doesn't match minimum model requirement");

    // Compute the plane parameters.
    double x0 = data[0].x;
    double y0 = data[0].y;
    double z0 = data[0].z;

    double x1 = data[1].x;
    double y1 = data[1].y;
    double z1 = data[1].z;

    double x2 = data[2].x;
    double y2 = data[2].y;
    double z2 = data[2].z;

    // TODO: Calculate the parameter for 3D plane parameter.
    plane_a = (x0 + y0);
    plane_b = (x1 + y1);
    plane_c = -1.0;
    plane_d = (x2 + y2);

    plane_denominator = sqrt(plane_a * plane_a + plane_b * plane_b + plane_c * plane_c);
}