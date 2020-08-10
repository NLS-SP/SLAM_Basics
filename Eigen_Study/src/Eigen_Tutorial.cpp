//
// Created by Robotics_qi on 2020/8/7.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main()
{
    // Cout the Eigen Version.
    std::cout << "Eigen Version: " << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;

    // Create the Matrix
    Eigen::Matrix3f A; Eigen::Matrix4d B;

    // Set Each Coefficient to a uniform random value in the range[-1, 1]
    A = Eigen::Matrix3f::Random();

    // Set B to the Identity Matrix.
    B = Eigen::Matrix4d::Identity();

    // Set all elements to zero.
    A = Eigen::Matrix3f::Zero();

    // Set all elements to ones.
    A = Eigen::Matrix3f::Ones();

    // Set all elements to a constant values.
    B = Eigen::Matrix4d::Constant(4.5);

    // Matrix Operations.
    Eigen::Matrix4f M1 = Eigen::Matrix4f::Constant(2);
    Eigen::Matrix4f M2 = Eigen::Matrix4f::Constant(2.2);

    // Addition.
//    std::cout << M1 + M2 << std::endl;

    // Matrix multiplication.
//    std::cout << M1 * M2 << std::endl;

    // Scalar Multiplication, and subtraction.
//    std::cout << M2 - Eigen::Matrix4f::Ones() * 2.2 << std::endl;

    // Transposition.
//    std::cout << M1.transpose() << std::endl;

    // Inversion.
    // If Matrix is not invertible, generate NaNs.
//    std::cout << M1.inverse() << std::endl;

    // Square each element of the matrix.
    std::cout << M1.array().square() << std::endl;

    // Utility functions.
    Eigen::Vector3f v1 = Eigen::Vector3f::Ones();
    Eigen::Vector3f v2 = Eigen::Vector3f::Zero();
    Eigen::Vector4d v3 = Eigen::Vector4d::Random();
    Eigen::Vector4d v4 = Eigen::Vector4d::Constant(1.8);

    // Addition
    std::cout << v1 + v2 << std::endl << std::endl;
    // Subtraction.
    std::cout << v4 - v3 << std::endl;

    // Scalar Multiplication.
    std::cout << v4 * 2 << std::endl;

    // Equality.
    std::cout << (Eigen::Vector2f::Ones() * 3 == Eigen::Vector2f::Constant(3)) << std::endl;

    Eigen::Vector4f v5 = Eigen::Vector4f(1.0f, 2.0f, 3.0f, 4.0f);

    std::cout << Eigen::Matrix4f::Random() * v5 << std::endl;

    v1 = Vector3f::Random();
    v2 = Vector3f::Random();
    std::cout << v1 * v2.transpose() << std::endl;

    std::cout << v1.dot(v2) << std::endl << std::endl;
    std::cout << v1.normalized() << std::endl << std::endl;
    std::cout << v1.cross(v2) << std::endl;

    // Convert a vector to and from homogenous coordinates.
    Eigen::Vector3f s = Eigen::Vector3f::Random();
    Eigen::Vector4f q = s.homogeneous();
    std::cout << (s == q.hnormalized()) << std::endl;


}