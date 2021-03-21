//
// Created by Robotics_qi on 2021/3/7.
//

#include <chrono>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

#include <ceres/ceres.h>

#include "pnp_optimization.h"

using namespace std;
using namespace cv;

using ceres::CostFunction;
using ceres::Problem;
using ceres::SizedCostFunction;
using ceres::Solve;
using ceres::Solver;

void find_feature_matches(const cv::Mat &image_1,
                          const cv::Mat &image_2,
                          std::vector<cv::KeyPoint> &keypoints_1,
                          std::vector<cv::KeyPoint> &keypoints_2,
                          std::vector<cv::DMatch> &matches);
// 像素坐标转相机归一化坐标系
Point2d pixel2cam(const Point2d &p, const Mat &K);
Mat K = (Mat_<double>(3, 3) << 520.9, 0.0, 325.1, 0.0, 521.0, 249.7, 0, 0, 1);


/**********************************************************************
 * !@brief 创建基于CERES的PNP-BA的优化方程.
 *  @param 2 表示的是残差的维度是2维
 *  @param 7 表示的是待优化的参数是7维度
 *********************************************************************/
class PNPAnalyticalCostFunction : public ceres::SizedCostFunction<2, 7>{
public:
    // 约束方程的构建
    PNPAnalyticalCostFunction(Point2f uv, Point3f xyz) : _uv(uv), _xyz(xyz){}

    virtual ~PNPAnalyticalCostFunction(){}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{
        Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
        Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
        Eigen::Vector3d point = Eigen::Vector3d{double(_xyz.x),double(_xyz.y),double(_xyz.z)};
        Eigen::Vector3d point_in_cam;
        point_in_cam = q_last_curr * point + t_last_curr;
        double fx = K.at<double>(0, 0);
        double fy = K.at<double>(1, 1);
        double xp = point_in_cam[0] / point_in_cam[2];
        double yp = point_in_cam[1] / point_in_cam[2];
        double inv_z = 1.0 / point_in_cam[2];
        double inv_z2 = inv_z * inv_z;
        double u_ = xp * K.at<double>(0, 0) + K.at<double>(0, 2);
        double v_ = yp * K.at<double>(1, 1) + K.at<double>(1, 2);
        Eigen::Matrix<double, 2, 3> temp_jacobian_matrix;
        temp_jacobian_matrix.setZero();
        temp_jacobian_matrix(0, 0) = -fx * inv_z;
        temp_jacobian_matrix(0, 2) = fx * point_in_cam[0] * inv_z2;
        temp_jacobian_matrix(1, 1) = -fy * inv_z;
        temp_jacobian_matrix(1, 2) = fy * point_in_cam[1] * inv_z2;
        residuals[0] = double(_uv. x) - u_;
        residuals[1] = double(_uv.y) - v_;

        if(jacobians != NULL){
            if(jacobians[0] != NULL){
                Eigen::Map<Eigen::Matrix<double, 2, 6>> J_Point(jacobians[0]);
                Eigen::Matrix3d skew_point = skew(point_in_cam);
                Eigen::Matrix<double, 3, 6> dp_by_so3;
                dp_by_so3.block<3, 3>(0, 0) = -skew_point;
                dp_by_so3.block<3, 3>(0, 3).setIdentity();
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > J_SE3(jacobians[0]);
                J_SE3.setZero();
                J_SE3.block<2, 6>(0, 0) = temp_jacobian_matrix * dp_by_so3;
            }
        }
        return true;
    }
public:
    // 观测量，像素u,v和3D特征点x,y,z
    Point2f _uv;
    Point3f _xyz;
};


int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    if(argc != 5){
        std::cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2." << std::endl;
        return 1;
    }
    // The array used for ceres optimization adjusting.
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    // TODO: The Eigen Map function.
    Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
    Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

    // Read image1 and image2.
    cv::Mat image_1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat image_2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(image_1.data && image_2.data && "Cannot load images!");

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(image_1, image_2, keypoints_1, keypoints_2, matches);
    std::cout << "Find " << matches.size() << " matches!" << std::endl;

    // 建立3D点！
    cv::Mat d1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for(DMatch m : matches){
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if(d == 0) continue;
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }
    std::cout << "3D-2D Paris: " << pts_3d.size() << std::endl;

    Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false);
    cv::Mat R;
    cv::Rodrigues(r, R);

    std::cout << "==============================================================" << std::endl;
    std::cout << "The Rotation from PNP is: " << std::endl << R  << std::endl;
    std::cout << "==============================================================" << std::endl;
    std::cout << "The Translation from PNP is: " << std::endl << t << std::endl;
    std::cout << "==============================================================" << std::endl;

    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
    for(int i = 0; i < pts_2d.size(); ++i){
        ceres::CostFunction *cost_function = new PNPAnalyticalCostFunction(pts_2d[i], pts_3d[i]);
        problem.AddResidualBlock(cost_function, NULL, parameters);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << "==============================================================" << std::endl;
    std::cout << "The Rotation is: " << std::endl << q_w_curr.toRotationMatrix()  << std::endl;
    std::cout << "==============================================================" << std::endl;
    std::cout << "The Translation is: " << std::endl << t_w_curr << std::endl;
    std::cout << "==============================================================" << std::endl;
}

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches){
    // 初始化
    Mat descriptors_1, descriptors_2;
    // Used in OpenCV3.
    Ptr<cv::FeatureDetector> detector = ORB::create();
    Ptr<cv::DescriptorExtractor> descriptor = ORB::create();

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    // 检测Fast角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // 第二步：依据角点位置来提取BRIEF描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    // 第三步: 用Hamming距离对两幅图像的描述子进行匹配.
    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);

    // 第四步：匹配点筛选
    double min_dist = 10000, max_dist = 0;

    // 找出所有匹配之间的最大或最小距离, 即是最相似或最不相似的距离
    for(int i = 0; i < descriptors_1.rows; ++i){
        double dist = match[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    std::cout << "The max distance is: " << max_dist << std::endl;
    std::cout << "The min distance is: " << min_dist << std::endl;

    // 当描述子距离大于两倍的最小距离之间，即认为匹配有误, 但有时候最小距离都会比较小，设置30为下限
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
