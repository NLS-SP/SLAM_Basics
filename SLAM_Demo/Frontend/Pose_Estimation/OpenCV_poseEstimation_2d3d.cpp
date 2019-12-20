#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <chrono>

using namespace std;
using namespace cv;

void find_feature_matches(
        const cv::Mat &img_1,
        const cv::Mat &img_2,
        std::vector<cv::KeyPoint> &keypoints_1;
        std::vector<cv::KeyPoint> &keypoints_2,
        std::vector<cv::DMatch> &matches);

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);

// Bundle adjustment by g2o
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

void bundleAdjustmentG2O(const VecVector3d &points_3d, const VecVector2d &points_2d, const cv::Mat &K, Sophus::SE3d &pose);

// Bundle Adjustment by gauss-newton
void bundleAdjustmentGaussNewton(const VecVector3d &points_3d, const VecVector2d &points_2d,
        const cv::Mat &K, Sophus::SE3 &pose);

int main(int argc, char **argv) {
    if (argc != 5) {
        std::cout << "Usage: pose_estimation_3d2d img1 img2 depth1 depth2" << std::endl;
        return 1;
    }

    // 读取图像
    cv::Mat img_1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    std::cout << "一共找到了" << matches.size() << "组匹配点." << std::endl;

    // 建立3D点
    cv::Mat d1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 529.0, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;

    for(DMatch match:matches){
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[match.queryIdx].pt.y))[int(keypoints_1[match.queryIdx].pt.x)];
        if(d == 0) continue;                // bad depth
        float dd = d / 5000.0;
        cv::Point2d p1 = pixel2cam(keypoints_1[match.queryIdx].pt, K);
        pts_3d.push_back(cv::Point3f(p1.x * dd, p1.y *dd, dd));
        pts_2d.push_back(keypoints_2[match.trainIdx].pt);
    }
    std::cout << "3d-2d pairs: " << pts_3d.size() << std::endl;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    cv::Mat r, t;
    cv::solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false);
    cv::Mat R;
    cv::Rodrigues(r, R);    // 利用罗德里格斯公式，将旋转向量转换为旋转矩阵
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    std::cout << "Solve PNP in OpenCV cost " << time_used.count() << " seconds."  << std::endl;
    std::cout << "R = " << std::endl << R << std::endl;
    std::cout << "T = " << std::endl << t << std::endl;

    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for(size_t i = 0; i < pts_3d.size(); ++i){
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
    }

    std:cout << "Calling bundle adjustment by gauss newton" << std::endl;
    Sophus::SE3d pose_gn;
    t1 = chrono::steady_clock::now();
    bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    std::cout << "Solve PNP by gauss-newton cost " << time_used.count() << " seconds" << std::endl;

    std::cout << "Calling bundle adjustment by g2o" << std::endl;
    Sophus::SE3d pose_g2o;
    t1 = chrono::steady_clock::now();
    bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    std::cout << "Solve PNP by g2o cost " << time_used.count() << " seconds." << std::endl;
    return 0;
}

void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2,
        std::vector<cv::KeyPoint> &keypoints_1,
        std::vector<cv::KeyPoint> &keypoints_2,
        std::vector<cv::DMatch> &matches){

    // 1. 初始化
    cv::Mat descriptors_1, descriptors_2;
    Ptr<cv::FeatureDetector> detector = ORB::create();
    Ptr<cv::DescriptorExtractor> descriptor = ORB::create();
    Ptr<cv::DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // 第一步:检测角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //第二步：根据角点位置计算描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    // 第三步：对两幅图像进行匹配
    std::vector<cv::DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);

    // 第四步： 匹配点筛选
    double min_dist = 10000, max_dist = 0;

    for(int i = 0; i < descriptors_1.rows; i++){
        double dist = match[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    for(int i = 0; i < descriptors_1.rows; i++)
        if(match[i].distance <= std::max(2*min_dist, 30.0))
            matches.push_back(match[i]);

}

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K){
    return cv::Point2d(
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1. 1)
            );
}

void bundleAdjustmentGaussNewto(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K, Sophus::SE3 &pose){
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    const int iteration = 10;
    double cost = 0, lastCost = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    for(int iter = 0; iter < iteration; iter++){
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute the cost
        for(int i = 0; i < points_3d.size(); i++){
            Eigen::Vector3d pc = pose * points_3d[i];
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);

            Eigen::Vector2d e = points_2d[i] - proj;

            cost += e.squaredNorm();
            Eigen::Matrix<double, 2, 6> J;
            J << -fx * inv_z, 0, fx * pc[0] *inv_z2, fx * pc[0] * pc[1] * inv_z2,
            -fx -fx * pc[0] * pc[0] * inv_z2, fx * pc[1] * inv_z, 0,
            -fy * inv_z,
            fy * pc[1] * inv_z,
            fy + fy * pc[1] * pc[1] * inv_z2,
            -fy * pc[0] * pc[1] * inv_z2,
            -fy * pc[0] * inv_z;

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        Vector6d dx;
        dx = H.ldlt().solve(b);

        if(iter > 0 && cost > lastCost) {
            std::cout << "Cost: " << cost << ", last cost: " << lastCost << std::endl;
            break;
        }

        pose = Sophus::SE3d::exp(dx) * pose;
        lastCost = cost;

        std::cout << "iteration: " << iter << " cost=" << cout.precision(12) << cost << std::endl;
        if(dx.norm() < 1e-6)
            break;
    }

    std::cout << "Pose by Gauss-Newton: " << pose.matrix() << std::endl;

}