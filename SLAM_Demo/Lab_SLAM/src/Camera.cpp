//
// Created by gatsby on 2019-02-27.
//

#include "Lab_SLAM/Common.h"
#include "Lab_SLAM/Camera.h"

namespace Lab_SLAM{

    Camera::Camera(){}

    Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3 &T_c_w){
        return T_c_w * p_w;
    }

    Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3 &T_c_w){
        return T_c_w.inverse() * p_c;
    }

    Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d &p_c){
        return Eigen::Vector2d(fx_ * p_c(0,0) / p_c(2, 0) + cx_, fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
    }

    Eigen::Vector2d Camera::pixel2camera(const Eigen::Vector2d &p_p, double depth){
        return Eigen::Vector3d(
                (p_p(0, 0) - cx_) * depth / fx_,
                (p_p(1, 0) - cy_) * depth / fy_, depth);
    }

    Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3 &T_c_w){
        return camera2pixel(world2camera(p_w, T_c_w));
    }

    Eigen::Vector3d Camera::pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3 &T_c_w, double depth){
        return camera2world(pixel2camera(p_p, depth), T_c_w);
    }
}