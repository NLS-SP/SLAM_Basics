#include <ICP.h>
#include <numeric>
#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

float dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb) {
    return sqrt((pta[0] - ptb[0]) * (pta[0] - ptb[0]) + (pta[1] - ptb[1]) * (pta[1] - ptb[1]) +
                (pta[2] - ptb[2]) * (pta[2] - ptb[2]));
}

Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
    Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4, 4);
    Eigen::Vector3d centroid_A(0, 0, 0);
    Eigen::Vector3d centroid_B(0, 0, 0);
    Eigen::MatrixXd AA = A; Eigen::MatrixXd BB = B;
    int row = A.rows();
    for(int i = 0; i < row; ++i){
        centroid_A += A.block<1, 3>(i, 0).transpose();
        centroid_B += B.block<1, 3>(i, 0).transpose();
    }

    centroid_A /= row;
    centroid_B /= row;

    for(int i = 0; i < row; ++i){
        AA.block<1, 3>(i, 0) = A.block<1, 3>(i, 0) - centroid_A.transpose();
        BB.block<1, 3>(i, 0) = B.block<1, 3>(i, 0) - centroid_B.transpose();
    }
    Eigen::MatrixXd H = AA.transpose() * BB;
    Eigen::MatrixXd U;
    Eigen::VectorXd S;
    Eigen::MatrixXd V;
    Eigen::MatrixXd Vt;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, ComputeFullU | ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();
    Vt = V.transpose();

    R = V * U.transpose();
    if(R.determinant() < 0){
        Vt.block<1, 3>(2, 0) *= -1;
        R = Vt.transpose() * U.transpose();
    }
    t = centroid_B - R * centroid_A;

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

NEIGHBOR nearest_neighbor(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst){
    int row_src = src.rows();
    int row_dst = src.rows();
    Eigen::Vector3d vec_src;
    Eigen::Vector3d vec_dst;
    NEIGHBOR neigh;
    float min = 100;
    float index = 0;
    float dist_temp = 0.0;

    for(int i = 0; i < row_src; ++i){
        min = 100; index = 0;dist_temp = 0;
        vec_src = src.block<1, 3>(i, 0).transpose();
        for(int j = 0; j < row_dst; ++j){
            vec_dst = dst.block<1,3>(j,0).transpose();
            dist_temp = dist(vec_src, vec_dst);
            if(dist_temp < min){
                min = dist_temp;
                index = j;
            }
        }
        neigh.distances.push_back(min);
        neigh.indices.push_back(index);
    }
    return neigh;
}

ICP_OUT icp(const Eigen::MatrixXd &A, const Eigen::MatrixXd& B, int max_iterations, int tolerance){
    int row = A.rows();
    Eigen::MatrixXd src = Eigen::MatrixXd::Ones(3+1, row);
    Eigen::MatrixXd src3d = Eigen::MatrixXd::Ones(3, row);
    Eigen::MatrixXd dst = Eigen::MatrixXd::Ones(3+1, row);
    NEIGHBOR neighbor;
    Eigen::Matrix4d T;
    Eigen::Matrix4d dst_chorder = Eigen::MatrixXd::Ones(3, row);
    ICP_OUT result;
    int iter = 0;

    for(int i = 0; i < row; i++){
        src.block<3, 1>(0, i) = A.block<1,3>(i,0).transpose();
        src3d.block<3,1>(0, i) = A.block<1,3>(i, 0).transpose();
        dst.block<3,1>(0, i) = B.block<1,3>(i,0).transpose();
    }

    double prev_error = 0.0;
    double mean_error = 0.0;
    for(int i = 0; i < max_iterations; ++i){
        neighbor = nearest_neighbor(src3d.transpose(), B);
        for(int j = 0; j < row; j++)
            dst_chorder.block<3,1>(0,j) = dst.block<3,1>(0, neighbor.indices[j]);
        T = best_fit_transform(src3d.transpose(), dst_chorder.transpose());
        src = T * src;
        for(int j = 0; j < row; j++)
            src3d.block<3,1>(0,j) = src.block<3,1>(0,j);

        mean_error = std::accumulate(neighbor.distances.begin(), neighbor.distances.end(),0.0)/ neighbor.distances.size();
        if(abs(prev_error - mean_error) < tolerance) break;
        prev_error = mean_error;
        iter = i + 2;
    }
    T = best_fit_transform(A, src3d.transpose());
    result.trans = T;
    result.distances = neighbor.distances;
    result.iter = iter;

    return result;
}