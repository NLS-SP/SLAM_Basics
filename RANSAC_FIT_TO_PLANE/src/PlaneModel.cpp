//
// Created by Robotics_qi on 2020/10/8.
//

#include <PlaneModel.h>


void PlaneModel::parameter_estimate(const std::vector<Point3D> &fitting_points) {
    std::vector<Eigen::Vector3d> plane_pts;
    for(int i = 0; i < fitting_points.size(); ++i){
        Eigen::Vector3d pt(fitting_points[i].x, fitting_points[i].y, fitting_points[i].z);
        plane_pts.push_back(pt);
    }
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for(const auto &pt : plane_pts) center += pt;
    center /= plane_pts.size();

    Eigen::MatrixXd A(plane_pts.size(), 3);
    for(int i = 0; i < plane_pts.size(); ++i){
        A(i, 0) = plane_pts[i][0] - center[0];
        A(i, 1) = plane_pts[i][1] - center[1];
        A(i, 2) = plane_pts[i][2] - center[2];
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
    plane_a = svd.matrixV()(0, 2);
    plane_b = svd.matrixV()(1, 2);
    plane_c = svd.matrixV()(2, 2);
    plane_d = -(plane_a * center[0] + plane_b * center[1] + plane_c * center[2]);
}