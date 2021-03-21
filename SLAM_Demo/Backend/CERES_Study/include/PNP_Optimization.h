//
// Created by Robotics_qi on 2021/3/20.
//

#ifndef SLAM_BACKEND_PNP_OPTIMIZATION_H
#define SLAM_BACKEND_PNP_OPTIMIZATION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

inline Eigen::Matrix3d skew(Eigen::Vector3d &mat_in){
    Eigen::Matrix<double, 3, 3> skew_mat;
    skew_mat.setZero();
    skew_mat(0, 1) = -mat_in(2);
    skew_mat(0, 2) = mat_in(1);
    skew_mat(1, 2) = -mat_in(0);
    skew_mat(1, 0) = mat_in(2);
    skew_mat(2, 0) = -mat_in(1);
    skew_mat(2, 1) = mat_in(0);
    return skew_mat;
}

void GetTransformFromSE3(const Eigen::Matrix<double, 6, 1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t){
    Eigen::Vector3d omega(se3.data());
    Eigen::Vector3d upsilon(se3.data() + 3);
    Eigen::Matrix3d Omega = skew(omega);

    double theta = omega.norm();
    double half_theta = 0.5 * theta;

    double image_factor;
    double real_factor = cos(half_theta);
    if (theta < 1e-10) {
        double theta_sq = theta * theta;
        double theta_po4 = theta_sq * theta_sq;
        image_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
    } else {
        double sin_half_theta = sin(half_theta);
        image_factor = sin_half_theta / theta;
    }

    q = Eigen::Quaterniond(real_factor, image_factor * omega.x(), image_factor * omega.y(),
                           image_factor * omega.z());

    // 这里需要看下四元数的公式和矩阵对应
    Eigen::Matrix3d J;
    if (theta < 1e-10) {
        J = q.matrix();
    } else {
        Eigen::Matrix3d Omega2 = Omega * Omega;
        J = (Eigen::Matrix3d::Identity() + (1 - cos(theta)) / (theta * theta) * Omega +
             (theta - sin(theta)) / (pow(theta, 3)) * Omega2);
    }

    t = J * upsilon;
}

class PoseSE3Parameterization : public ceres::LocalParameterization{
public:
    PoseSE3Parameterization(){}

    virtual ~PoseSE3Parameterization(){}

    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const{
        Eigen::Map<const Eigen::Vector3d> trans(x+4);
        Eigen::Quaterniond delta_q;
        Eigen::Vector3d delta_t;

        GetTransformFromSE3(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(delta), delta_q, delta_t);
        Eigen::Map<const Eigen::Quaterniond> quater(x);
        Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
        Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

        quater_plus = delta_q * quater;
        trans_plus = delta_q * trans + delta_t;
        return true;
    }

    virtual bool ComputeJacobian(const double *x, double *jacobian) const{
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        (j.topRows(6)).setIdentity();
        (j.bottomRows(1)).setZero();
        return true;
    }

    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize()  const { return 6; }
};

#endif //SLAM_BACKEND_PNP_OPTIMIZATION_H
