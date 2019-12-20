//
// Created by gatsby on 2019-02-13.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <chrono>
#include <sophus/se3.hpp>

using namespace std;
using namespace cv;

void find_feature_matches(
        const cv::Mat &img_1, const cv::Mat &img_2,
        std::vector<cv::KeyPoint> &keypoints_1,
        std::vector<cv::KeyPoint> &keypoints_2,
        std::vector<cv::DMatch> &matches);

//像素坐标转化成相机归一化坐标
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);

void pose_estimation_3d3d(
        const std::vector<cv::Point3f>  &pts1,
        const std::vector<cv::Point3f>  &pts2,);

void bundleAdjustment(
        const std::vector<cv::Point3f> &points_3d,
        const std::vector<cv::Point3f> &points_2d,
        cv::Mat &R, cv::Mat &t);

// vertex and edges used in g2o ba
class VertexPose : public g2o:BaseVertex<6, Sophus::SE3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() overried{ _estimate = Sophus::SE3d; }

    // left multiplication on SE3
    virtual void oplusImpl(const double *update) override{
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sohpus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(istream &in) override{}
    virtual bool write(ostream &out) const override{}
};

// g2o edge
class edgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    edgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point){}

	virtual void computeError() override{
		const VertexPose *pose = static_cast<const VertexPose*>(_vertices[0]);
		_error = _measurement - pose->estimate() * _point;
	}

	virtual void linearizeOplus() override{
		VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
		Sophus::SE3d T = pose->estimate();
		Eigen::Vector3d xyz_trans = T * _point;
		_jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
		_jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
	}

	bool read(istream &in){}
	bool write(ostream &out) const{}
	
protected:
	Eigen::Vector3d _point;
};


int main(int argc, char **argv)
{
    if(argc != 5){
        std::cout << "Usage: pose_estimation_3d3d img1 img2 depth1 depth2" << std::endl;
        return 1;
    }

    // 读取图像
    cv::Mat img_1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    std::cout << "一共找到了" << matches.size() << "组匹配点" << std::endl

    // 建立3D点
    cv::Mat depth1 = cv::imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat depth2 = cv::imread(argv[4], CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point3f> pts1, pts2;

    for(auto match : matches){
        ushort d1 = depth1.ptr<unsigned short>(int(keypoints_1[match.queryIdx].pt.y))[int(keypoints_1[match.queryIdx].pt.x)];
        ushort d2 = depth2.ptr<unsigned short>(int(keypoints_2[match.queryIdx].pt.y))[int(keypoints_2[match.queryIdx].pt.x)];

        if(d1 == 0 || d2 == 0) continue;    // exliminate the bad depth

        cv::Point2d p1 = pixel2cam(keypoints_1[match.queryIdx].pt, K);
        cv::Point2d p2 = pixel2cam(keypoints_2[match.trainIdx].pt, K);

        float dd1 = float(d1) / 5000.0;
        float dd2 = float(d2) / 5000.0;
        pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));
        pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }

    std::cout << "3d-2d pairs: " << pts1.size() << std::endl;
    cv::Mat R, t;
    pose_estimation_3d3d(pts1, pts2, R, t);
    std::cout << "ICP via SVD results: " << std::endl;
    std::cout << "R = " << R << std::endl;
    std::cout << "t = " << t << std::endl;
    std::cout << "R_inv = " << R.t() << std::endl;
    std::cout << "t_inv = " << -R.t() * t << std::endl;

    std::cout << "calling bundle adjustment" << std::endl;
    bundleAdjustment(pts1, pts2, R, t);

    // verify p1 = R*p2 + t
    for(int i = 0; i < 5; i++){
        std::cout << "p1 = " << pts1[i] << std::endl;
        std::cout << "p2 = " << pts2[i] << std::endl;
        std::cout << "(R*p2=t) = " << R * (Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t << std::endl;
        std::cout << std::endl;
    }
}

void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2,
                          std::vector<cv::KeyPoint> &keypoints_1,
                          std::vector<cv::KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches){
    // 初始化
    cv::Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    cv::Ptr<cv::FeatureDetector>  detector = ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = ORB::create();

    cv::Ptr<cv::DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    // 第一步:检测Oriented Fast角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // 第二步：根据角点位置计算Brief描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    // 第三步：根据描述子来进行Hamming匹配
    std::vector<cv::DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);

    // 第四步：匹配点筛选
    double min_dist = 10000, max_dist = 0;

    // 找出所有匹配中的最大距离和最小距离
    for(int i = 0; i < descriptors_1.rows; i++){
        double dist = match[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }
    printf("-- Max dist: %f \n", max_dist);
    printf("-- Min dist: %f \n", min_dist);

    //当前描述子之间的匹配距离大于两倍的匹配最小距离时，就判断为匹配有误
    for(int i = 0; i < descriptors_1.rows; i++){
        if(match[i].distance <= max(2*min_dist, 30.0))
            matches.push_back(match[i]);
    }
}

Point2d pixel2cam(const Point2d &p, const Mat &K){
    return Point2d(
            (p.x - K.at<double>(0, 2) / K.at<double>(0, 0)),
            (p.y - K.at<double>(1, 2) / K.at<double>(1, 1))
            );
}

void pose_estimation_3d3d(const vector<cv::Point3f> &pts_1,
                          const vector<cv::Point3f> &pts_2,
                          Mat &R, Mat &t){
    cv::Point3f p1, p2;
    int N = pts_1.size();
    for(int i = 0; i < N; i++){
        p1 += pts_1[i];
        p2 += pts_2[i];
    }

    p1 = cv::Point3f(Vec3f(p1) / N);
    p2 = cv::Point3f(Vec3f(p2) / N);
    vector<cv::Point3f> q1(N), q2(N);
    for(int i = 0; i < N; i++){
        q1[i] = pts_1[i] - p1;
        q2[i] = pts_2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i = 0; i < N; i++)
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    std::cout << "W=" << W << std::endl;

    // SVD on w
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    std::cout << "U=" << U << std::endl;
    std::cout << "V=" << V << std::endl;

    Eigen::Matrix3d R_ = U * (V.transpose());
    if(R_.determinant() < 0)
        R_ = -R_;

    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

    R = (cv::Mat_<double>(3, 3) <<
            R_(0, 0), R_(0, 1), R_(0, 2),
            R_(1, 0), R_(1, 1), R_(1, 2),
            R_(2, 0), R_(2, 1), R_(2, 2));

    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}