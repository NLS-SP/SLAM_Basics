#include <iostream>
#include <cstdlib>
#include <ctime>
#include <Eigen/Eigen>

Eigen::Isometry3d icpPoint2Point(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &reference,
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &current){
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    if(reference.size() != current.size()) return transform;
    int N = reference.size();
    Eigen::Map<Eigen::Matrix3Xd> ps(&reference[0].x(), 3, N);
    Eigen::Map<Eigen::Matrix3Xd> qs(&current[0].x(), 3, N);
    Eigen::Vector3d p_dash = ps.rowwise().mean();
    Eigen::Vector3d q_dash = qs.rowwise().mean();
    Eigen::Matrix3Xd ps_centered = ps.colwise() - p_dash;
    Eigen::Matrix3Xd qs_centered = qs.colwise() - q_dash;
    Eigen::Matrix3Xd K = qs_centered * ps_centered.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(K, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if(R.determinant() < 0) R.col(2) *= -1;

    transform.linear() = R;
    transform.translation() = q_dash - R * p_dash;
    return transform;
}