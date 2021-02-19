#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace cv;
using namespace std;

int main()
{
    // Create SIFT class Pointer.
    cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
    // 读入图片
    cv::Mat img_1 = cv::imread("/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Features/000888.png");
    cv::Mat img_2 = cv::imread("/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Features/000896.png");

    // Detect the keypoints.
    vector<KeyPoint> keypoints_1, keypoints_2;
    f2d->detect(img_1, keypoints_1);
    f2d->detect(img_2, keypoints_2);
    //Calculate descriptors (feature vectors)
    cv::Mat descriptors1;  // = cv::Mat(img_1.rows, img_1.cols, CV_32F);
    cv::Mat descriptors2;  // = cv::Mat(img_2.rows, img_2.cols, CV_32F);
    f2d->compute(img_1, keypoints_1, descriptors1);
    f2d->compute(img_2, keypoints_2, descriptors2);
    //Matching descriptor vector using BFMatcher
    BFMatcher matcher;
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    double min_dist = 10000, max_dist = 0;

    for(int i = 0; i < descriptors1.rows; i++){
        double dist = matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    min_dist = min_element(matches.begin(), matches.end(), [](const DMatch &m1, const DMatch &m2){
        return m1.distance < m2.distance;
    })->distance;
    max_dist = min_element(matches.begin(), matches.end(), [](const DMatch &m1, const DMatch &m2){
        return m1.distance > m2.distance;
    })->distance;

    std::cout << "The max distance: " << max_dist << std::endl;
    std::cout << "The min distance: " << min_dist << std::endl;
    std::vector<DMatch> good_matches;
    for(int i = 0; i < descriptors1.rows; i++)
        if(matches[i].distance <= max(2*min_dist, 30.0))
            good_matches.push_back(matches[i]);

    //绘制匹配出的关键点
    Mat img_matches;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_matches);
    imshow("Matching", img_matches);
    cv::waitKey(0);
}