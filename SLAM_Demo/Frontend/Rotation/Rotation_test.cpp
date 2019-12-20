//
// Created by Robotics_qi on 2019-11-18.
//

#include <iostream>
#include <Eigen/Dense>
#include "sophus/so3.hpp"

int main()
{
    // 首先创建旋转轴和旋转角度
    Eigen::AngleAxisd t_V(M_PI/4, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d t_R = t_V.matrix();
    Eigen::Quaterniond t_Q(t_V);
    t_Q.normalize();

    // 分别输出看一看
    std::cout << "The Rotation Matrix is: " << std::endl << t_R << std::endl;
    std::cout << "The quaternion is: " << std::endl << t_Q.coeffs() << std::endl;
    std::cout << "The Rotation from quaternion is: " << std::endl << t_Q.matrix() << std::endl;

    // The update state is w = [0.01, 0.02, 0.03]
    Eigen::Vector3d vector_update(0.01, 0.02, 0.03);
    Eigen::Matrix3d Matrix_update;
    Matrix_update << 0, -vector_update[2],vector_update[1],
                     vector_update[2], 0, -vector_update[0],
                     -vector_update[1],vector_update[0], 0;


    // The transformed Rotation represented in Rotation Matrix is:
    Eigen::Matrix3d R_updated;
    Matrix_update = Matrix_update.array().exp();
    R_updated = t_R * Matrix_update;

    Eigen::Quaterniond Q_update(1, 1/2*vector_update[0], 1/2*vector_update[1], 1/2*vector_update[2]);
    Eigen::Quaterniond Q_updated;
    Q_updated = t_Q * Q_update;

    std::cout << "After update, The represented in rotation Matrix is: " << std::endl
              << R_updated << std::endl;

    Q_updated.normalize();
    std::cout << "After update, The represented in quaternion matrix is: " << std::endl
              << Q_updated.matrix() << std::endl;

}
