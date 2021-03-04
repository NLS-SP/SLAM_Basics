//
// Created by gatsby on 2019-02-20.
//

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    double ar = 1.0, br =  2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;

    int N = 100;
    double w_sigma = 1.0;
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;

    std::vector<double> x_data, y_data;
    for(int i = 0; i < N; ++i){
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    // 开始GaussNewton迭代.
    int iterations = 100;
    double cost = 0, lastCost = 0;
    for(int iter = 0; iter < iterations; iter++){
        Matrix3d H = Matrix3d::Zero();
        Vector3d b = Vector3d::Zero();

        for(int i = 0; i < N; ++i){
            double xi = x_data[i], yi = y_data[i];      // 第i个数据点
            double error = yi - exp(ae * xi * xi + be * xi + ce);
            Vector3d J; // 雅克比矩阵
            J[0] = - xi * xi * exp(ae * xi * xi + be * xi + ce);
            J[1] = - xi * exp(ae * xi * xi + be * xi + ce);
            J[2] = -exp(ae * xi * xi + be * xi + ce);

            H +=  inv_sigma * inv_sigma * J * J.transpose();
            b += -inv_sigma * inv_sigma * error * J;

            cost += error * error;
        }

        // 求解线性方程Hx = b;
        Vector3d dx = H.ldlt().solve(b);
        if(std::isnan(dx[0])){
            std::cout << "Result is NAN!" << std::endl;
            break;
        }
        if(iter > 0 && cost >= lastCost){
            std::cout << "cost: " << cost << " >= last cost: " << lastCost << ", break." << std::endl;
            break;
        }
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;
    }

    std::cout << "The estimated abc = "<< ae << ", " << be << ", " << ce << " !"<< std::endl;
}