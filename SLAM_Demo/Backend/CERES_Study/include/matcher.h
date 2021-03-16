//
// Created by Robotics_qi on 2021/3/7.
//

#ifndef SLAM_BACKEND_MATCHER_H
#define SLAM_BACKEND_MATCHER_H

#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class Matcher{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Matcher(cv::Mat& img1, cv::Mat& img2, cv::Mat& depth1);

    void extractORB(cv::Mat& im, std::vector<cv::KeyPoint>& kpts,
                    cv::Mat& descriptors);

    void find_feature_matches();

    void construct3d_to_2d(std::vector<cv::DMatch> &matches, cv::Mat &depth);

    cv::Mat getCameraK();

    cv::Point2d pixel2Cam(const cv::Point2d& p);

    virtual ~Matcher();

public:

    std::vector<cv::KeyPoint> kpts1;
    std::vector<cv::KeyPoint> kpts2;
    cv::Mat descriptors1;
    cv::Mat descriptors2;

    std::vector<cv::DMatch> good_matches;
    std::vector<cv::DMatch> matches;

    std::vector<cv::Point3d> pts_3d;
    std::vector<cv::Point2d> pts_2d;

private:
    // camera intrinsics.
    const float fx = 520.9;
    const float fy = 521.0;
    const float cx = 325.1;
    const float cy = 249.7;
    const float depthFactor = 5000.0;

    // ORB.
    cv::Ptr<cv::ORB> orb;
    // BFMatcher
    cv::Ptr<cv::BFMatcher> bfmatcher;
};

#endif //SLAM_BACKEND_MATCHER_H
