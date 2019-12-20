//
// Created by gatsby on 2019-02-13.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

/* *******************************
 * 本程序演示2D-2D的特征匹配估计相机运动
 * *******************************/

void find_feature_matches(
        const Mat &img_1, const Mat &img_2,
        std::vector<cv::KeyPoint> &keypoints_1,
        std::vector<cv::KeyPoint> &keypoints_2,
        std::vector<cv::DMatch> &matches);

void pose_estimation_2d2d(
        std::vector<cv::KeyPoint> keypoints_1,
        std::vector<cv::KeyPoint> keypoints_2,
        std::vector<cv::DMatch> matches,
        cv::Mat &R, cv::Mat &t);

// 相随坐标归一化
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);

int main(int argc, char **argv)
{
    if(argc != 3){
        std::cout << "Usage: pose_estimation_2d2d img1 img2" << std::endl;
        return 1;
    }

    // --load images:
    cv::Mat img_1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Can not load image!");

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    std::cout << "We have found: " << matches.size() << " matches." << endl;

    // -- 估计两张图像之间的运动
    cv::Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

    // -- 验证 E = t^R*Scale
    Mat t_x = (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
               t.at<double>(2, 0), 0, -t.at<double>(0, 0),
               -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    std::cout << "t^R= " << std::endl << t_x*R << std::endl;

    // -- 验证对极约束
    cv::Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for(cv::DMatch match : matches){
        cv::Point2d pt1 = pixel2cam(keypoints_1[match.queryIdx].pt, K);
        cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
        cv::Point2d pt2 = pixel2cam(keypoints_2[match.trainIdx].pt, K);
        cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
        cv::Mat d = y2.t() & t_x * R * y1;
        std::cout << "epipolar constraint = " << d << std::endl;
    }

    return 0;
}

void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2,
        std::vector<cv::KeyPoint> &keypoints_1,
        std::vector<cv::KeyPoint> &keypoints_2,
        std::vector<cv::DMatch> &matches){
    // 初始化
    cv::Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<cv::FeatureDetector> detector = ORB::create();
    Ptr<cv::DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // 第一步：检测Oriented FAST角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // 第二步：根据角点位置计算BRIEF描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    // 第三步: 对两幅图像中的BRIEF描述子进行匹配, 使用Hamming距离
    std::vector<cv::DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);

    // 匹配点筛选
    double min_dist = 10000, max_dist = 0;

    // 找出所有距离的最小距离和最大距离, 最相似和最不相似两组点之间的距离
    for(int i = 0; i < descriptors_1.rows; i++){
        double dist = match[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    std::cout << "-- Max dist: " << max_dist << std::endl;
    std::cout << "-- Min dist: " << min_dist << std::endl;

    // 当描述子之间的距离大于最小距离的两倍时，即认为匹配有误
    for(int i = 0; i < descriptors_1.rows; i++)
        if(match[i].distance <= std::max(2*min_dist, 30.0))
            matches.push_back(match[i]);
}

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K){
    return cv::Point2d(
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
            );
}

void pose_estimation_2d2d(std::vector<cv::KeyPoint> keypoints_1,
                          std::vector<cv::KeyPoint> keypoints_2,
                          std::vector<cv::DMatch> matches,
                          cv::Mat &R, cv::Mat &t){
    // 相机内参, TUM Freiburg2
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    // 把匹配点转换成std::vector<Point2f>的形式
    std::vector<Point2f> points_1;
    std::vector<Point2f> points_2;

    for(int i = 0; i < (int)matches.size(); i++){
        points_1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points_2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    // --计算基础矩阵
    cv::Mat fundamental_matrix;
    fundamental_matrix = cv::findFundamentalMat(points_1, points_2, CV_FM_8POINT);
    std::cout << "fundamental matrix is: " << std::endl << fundamental_matrix << std::endl;

    // --计算本质矩阵
    cv::Point2d principal_point(325.1, 249.7);                                  // 相机光心, TUM dataset标定值
    double focal_length = 521;                                                  // 相机焦距, TUM dataset标定值
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(points_1, points_2, focal_length, principal_point);

    // 计算单应矩阵
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(points_1, points_2, cv::RANSAC, 3);
    std::cout << "homography_matrix is: " << std::endl << homography_matrix << std::endl;

    cv::recoverPose(essential_matrix, points_1, points_2, R, t, focal_length, principal_point);
    std::cout << "R is " << std::endl << R << std::endl;
    std::cout << "t is " << std::endl << t << std::endl;
}
