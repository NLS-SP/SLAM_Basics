//
// Created by Robotics_qi on 2020/9/29.
//

#ifndef RANSAC_FIT_TO_PLANE_GEOMETRY_H
#define RANSAC_FIT_TO_PLANE_GEOMETRY_H
#include <iostream>

class Point2D{
public:
    Point2D(double px, double py) : x(px), y(py){}
    double x;
    double y;
};

inline std::ostream &operator<<(std::ostream &output, const Point2D &pnt){
    output << pnt.x << " " << pnt.y;
    return (output);
}

class Point3D{
public:
    Point3D():x(0.0),y(0.0),z(0.0){}
    Point3D(double px, double py, double pz) : x(px), y(py), z(pz){}
    double x;
    double y;
    double z;
};

inline std::ostream &operator<<(std::ostream &output, const Point3D &pnt){
    output << pnt.x << pnt.y << pnt.z;
    return (output);
}
#endif //RANSAC_FIT_TO_PLANE_GEOMETRY_H
