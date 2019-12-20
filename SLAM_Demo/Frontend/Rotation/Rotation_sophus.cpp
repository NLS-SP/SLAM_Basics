//
// Created by Robotics_qi on 2019-11-20.
//

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

using namespace std;

// 十四讲上的Sophus的示例代码
int main(int argc, char **argv)
{
    // 沿Z轴旋转90°的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    // 设置四元数
    Eigen::Quaterniond q(R);
    Sophus::SO3d SO3_R(R);                          // Sophus::SO3d直接可以用从旋转矩阵构建
    Sophus::SO3d SO3_Q(q);                          // Sophus::SO3d 也可以从Quaternion构建

    // 二者创建出来的结果是等价的
    std::cout << "The SO3 Created from the Eigen::Matrix is: " << std::endl
    << SO3_R.matrix() << std::endl;
    std::cout << "The SO3 Created from teh Eigen::Quaternion is: " << std::endl
    << SO3_R.matrix() << std::endl;
    std::cout << "They are equal!" << std::endl;

    // 使用对数映射得到李代数
    Eigen::Vector3d so3 = SO3_R.log();
    std::cout << "SO3 = " << so3.transpose() << std::endl;
    // hat 为向量到反对称矩阵
    std::cout << "SO3 hat = " << std::endl << Sophus::SO3d::hat(so3) << std::endl;
    // vee 为反对称到向量？这里啥意思，没弄懂
    std::cout << "So3 hat vee = " << std::endl
              << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << std::endl;

    // 增量扰动模型更新
    Eigen::Vector3d update_so3(1e-4, 0, 0);                          // 假设增量仅仅是这么多
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;       // 左扰动
    std::cout << "SO3 updated = " << std::endl << SO3_updated.matrix() << std::endl;

    std::cout << "********************对SE3进行测试**************************" << std::endl;
    Eigen::Vector3d t(1, 0, 0);                                      // 沿x轴平移1
    Sophus::SE3d SE3_Rt(R, t);                                              // 用R、t构建SE3
    Sophus::SE3d SE3_Qt(q, t);                                              // 用q、t构建SE3
    std::cout << "The SE3 build from the Rotation Matrix is: " << std::endl
              << SE3_Rt.matrix() << std::endl;
    std::cout << "The SE3 build from the Quaternion is: " << std::endl
              << SE3_Qt.matrix() << std::endl;
    std::cout << "They are same. " << std::endl;

    // 李代数se3 是一个六维向量, 为了方便，先typedef一下
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    std::cout << "SE3 = " << se3.transpose() << std::endl;
    // 同样，有hat和vee两个操作符
    std::cout << "SE3 hat = " << Sophus::SE3d::hat(se3) << std::endl;
    std::cout << "SE3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << std::endl;

    // SE3更新
    Vector6d update_se3;                // 更新变量
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    std::cout << "After Update, the SE3 matrix is: " << std::endl
              << SE3_updated.matrix() <<  std::endl;
}