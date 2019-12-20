#include <iostream>
#include <signal.h>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    // load images.
    Mat img_1 = imread("/Users/gatsby/Desktop/1.png");
    Mat img_2 = imread("./2.png");

    chrono::steady_clock::time_point t_detectStart = chrono::steady_clock::now();
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // 1. Detect the position of FAST keypoints.
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // 2. Compute the BRIEF descriptor with the position of keypoints
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    chrono::steady_clock::time_point t_detectEnd = chrono::steady_clock::now();
    chrono::duration<double> t_detectUsed = chrono::duration_cast<chrono::duration<double> >(t_detectEnd - t_detectStart);

    std::cout << "The detection cost: " << t_detectUsed.count() << std::endl;

    // 3. BFMatcher matcher(NORM HAMMING)
    chrono::steady_clock::time_point t_matchStart = chrono::steady_clock::now();
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);

    double min_dist = 10000, max_dist = 0;

    for(int i = 0; i < descriptors_1.rows; i++){
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
    for(int i = 0; i < descriptors_1.rows; i++)
        if(matches[i].distance <= max(2*min_dist, 30.0))
            good_matches.push_back(matches[i]);

    chrono::steady_clock::time_point t_matchEnd = chrono::steady_clock::now();
    chrono::duration<double> t_matchUsed = chrono::duration_cast<chrono::duration<double> >(t_matchEnd - t_matchStart);

    // 4. Show the result.
    cv::Mat img_match, img_keypoints;
    cv::drawKeypoints(img_1, keypoints_1, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("Features", img_keypoints);
    cv::imwrite("/Users/gatsby/Desktop/1_features.png", img_keypoints);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_match);


    std::cout << "The match costs: " << t_matchUsed.count() << std::endl;
    cv::imshow("", img_match);

    cv::waitKey(-1);
    return 0;
}