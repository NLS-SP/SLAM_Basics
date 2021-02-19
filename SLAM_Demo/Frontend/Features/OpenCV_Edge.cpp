//
// Created by Robotics_qi on 2021/2/3.
//

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"


#include <stdio.h>

using namespace cv;
using namespace std;



int main( int argc, const char** argv )
{
    std::string img_address = "/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Features/000000.png";
    cv::Mat image_src = imread(img_address, IMREAD_COLOR);
    cv::Mat image_copy = image_src; cv::Mat out;

    cv::namedWindow("原图");
    cv::imshow("原图", image_src);

    //!@brief 适用于OpenCV3以上版本的Canny检测算法
    cv::Mat dst, edge, gray;
    // 创建和image_src相同的图像矩阵
    dst.create(image_src.size(), image_src.type());
    // 将原图转换成灰度
    cv::cvtColor(image_src, gray, cv::COLOR_RGB2GRAY);
    // 滤波
    cv::blur(gray, edge, cv::Size(3, 3));
    cv::Canny(edge, out, 15, 10);

    dst = cv::Scalar::all(0);
    image_src.copyTo(dst, out);
    cv::namedWindow("Canny Feature");
    cv::imshow("Canny Feature", out);
    cv::waitKey();
    return 0;
}