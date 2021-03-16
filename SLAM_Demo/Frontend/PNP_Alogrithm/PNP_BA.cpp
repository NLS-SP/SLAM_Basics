//
// Created by Robotics_qi on 2021/3/16.
//

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <sophus/se3.hpp>

using namespace std;
using namespace cv;

void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2,
                          std::vector<cv::KeyPoint> &keypoints_1,
                          std::vector<cv::KeyPoint> &keypoints_2,
                          std::vector<cv::DMatch> &matches);

// 像素坐标转换为相机归一化坐标
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);


typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

void bundleAdjustmentG2O(const VecVector3d &points_3d,
                         const VecVector2d &points_2d,
                         const Mat &K,
                         Sophus::SE3d &pose);

void bundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K,
        Sophus::SE3d &pose
        );
int main(int argc, char **argv)
{
    if(argc != 5){
        std::cout << "USAGE: pose_estimation_3d2d img1 img2 depth1 depth2" << std::endl;
        return 1;
    }

    // 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    assert(img_1.data && img_2.data && "Cannot load images!");

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    std::cout << "一共找到了" << matches.size() << "组匹配点." << std::endl;

    // 建立3D点.
    cv::Mat d1 = cv::imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for(auto m : matches){
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if(d == 0)
            continue; // bad depth.
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }

    std::cout << "3D-2D pairs" << pts_3d.size() << std::endl;

    cv::Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false);
    cv::Mat R;
    cv::Rodrigues(r, R);

    std::cout << "R is: " << R << std::endl;
    std::cout << "t is: " << t << std::endl;

    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for(size_t i = 0; i < pts_3d.size(); ++i){
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
    }

    Sophus::SE3d pose_gn;
    bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
}

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          vector<KeyPoint> &keypoints_1,
                          vector<KeyPoint> &keypoints_2,
                          vector<DMatch> &matches){
    // 初始化
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // 第二步：根据角点位置计算BRIEF描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    // 第三步：对两幅图像中的BRIEF描述子进行匹配，使用Hamming距离
    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);

    // 第四步：匹配点筛选
    double min_dist = 100000, max_dist = 0;
    for(int i = 0; i < descriptors_1.rows; ++i){
        double dist = match[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    for(int i = 0; i < descriptors_1.rows; ++i)
        if(match[i].distance <= max(2 * min_dist, 30.0))
            matches.push_back(match[i]);
}

Point2d pixel2cam(const Point2d &p, const Mat &K){
    return Point2d(
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
            );
}

void bundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K,
        Sophus::SE3d &pose){

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    const int iterations = 10;
    double cost = 0, last_cost = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    for(int iter = 0; iter < iterations; ++iter){
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d  b = Vector6d::Zero();
        cost = 0;
        for(int i = 0; i < points_3d.size(); ++i){
            Eigen::Vector3d pc = pose * points_3d[i];
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);

            Eigen::Vector2d e = points_2d[i] - proj;

            cost += e.squaredNorm();
            Eigen::Matrix<double, 2, 6> J;
            J << -fx * inv_z, 0, fx * pc[0] * inv_z2, fx * pc[0] * pc[1] * inv_z2, -fx - fx * pc[0] * pc[0] * inv_z2,
            fx * pc[1] * inv_z, 0, -fy * inv_z, fy * pc[1] * inv_z2, fy + fy * pc[1] * pc[1] * inv_z2,
            -fy * pc[0] * pc[1] * inv_z2, -fy * pc[0] * inv_z;

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }
        Vector6d  dx;
        dx = H.ldlt().solve(b);

        pose = Sophus::SE3d::exp(dx) * pose;
        last_cost = cost;
    }
}