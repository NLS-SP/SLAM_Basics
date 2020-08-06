//
// Created by Robotics_qi on 2020/8/6.
//

#ifndef IMPLICIT_SURFACE_RECONSTRUCTION_ICP_H
#define IMPLICIT_SURFACE_RECONSTRUCTION_ICP_H

#include <Eigen/Eigen>
#include <vector>

#define N_PT 30
#define N_TESTS 100
#define NOISE_SIGMA 0.01
#define TRANSLATION 0.1
#define ROTATION 0.1

typedef struct{
    Eigen::Matrix4d trans;
    std::vector<float> distances;
    int iter;
} ICP_OUT;

typedef struct{
    std::vector<float> distances;
    std::vector<int> indices;
} NEIGHBOR;

Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
ICP_OUT icp(const Eigen::MatrixXd& A, const Eigen::MatrixXd &B, int max_iteration = 20, int tolerance = 0.00001);

// throughout method
NEIGHBOR nearest_neighbor(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst);
float dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb);

#endif //IMPLICIT_SURFACE_RECONSTRUCTION_ICP_H
