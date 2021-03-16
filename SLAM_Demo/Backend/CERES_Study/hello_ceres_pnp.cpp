//
// Created by Robotics_qi on 2021/3/7.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "matcher.h"
#include "reprojection_factor.h"

void PoseOptimization(const std::vector<cv::Point3d>& points_3d,
                      const std::vector<cv::Point2d>& points_2d,
                      cv::Mat &rvec, cv::Mat &t);

int main(int argc, char* argv[])
{
    if(argc < 4){
        std::cout << "USAGE:\n"
                  << "ceres_pnp img1 img2 depth1" << std::endl;
        exit(-1);
    }

    cv::Mat img1 = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat img2 = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat depth_1 = cv::imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);

    // Matcher match(img1, img2, depth1);
//    std::cout << "3D-2D pairs: " << match.pts_3d.size() << std::endl;

//    cv::Mat match.getCameraK();

    cv::Mat revc, t, R;

//    PoseOptimization(match.pts_3d, match.pts_2d, rvec, t);
}
