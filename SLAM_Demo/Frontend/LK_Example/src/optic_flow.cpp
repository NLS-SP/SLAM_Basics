//
// Created by Robotics_qi on 2021/1/29.
//
#include <chrono>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

std::string reference_image = "";
std::string target_image = "";

/// Optical Flow Tracker and Interfaces.
class OpticalFlowTracker{
public:
    OpticalFlowTracker(const Mat &img1_, const Mat &img2_, const std::vector<cv::KeyPoint> &kp1_, const std::vector<cv::KeyPoint> &kp2_,
                       std::vector<bool> &success_, bool inverse_ = true, bool has_initial_ = false):img1(img1_), img2(img2_),kp1(kp1_),
                       kp2(kp2_), success(success_), inverse(inverse_), has_initial(has_initial_){}

    void calculateOpticalFlow(const Range &range);

private:
    const cv::Mat &img1;
    const cv::Mat &img2;
    const std::vector<cv::KeyPoint> &kp1;
    const std::vector<cv::KeyPoint> &kp2;
    std::vector<bool> &success;
    bool inverse = true;
    bool has_initial = false;
};

void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false, bool has_initial = false);

//!@brief 这里表示的是双线性插值
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    // Check boundary.
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;
    if (y >= img.rows) y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] + (1 - xx) * yy * data[img.step] +
                 xx * yy * data[img.step + 1]);
}

int main()
{
    /// Read images and note they are CV_8UC1.
    cv::Mat img1 = cv::imread(reference_image, 0);
    cv::Mat img2 = cv::imread(target_image, 0);

    // extract key points, using GFTT.
    std::vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20);
    detector->detect(img1, kp1);

    // Track these key points in the second images.
    // First, using single level LK in the validation.
    std::vector<cv::KeyPoint> kp2_single;
    std::vector<bool> success_single;
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single);
}

void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse, bool has_initial) {
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
//    cv::Parallel_for_(Range(0, kp1.size()), std::bind(&OpticalFlowTracker::calculateOpticalFlow, &tracker, placeholders::_1));
    tracker.calculateOpticalFlow(Range(0, kp1.size()));
}

void OpticalFlowTracker::calculateOpticalFlow(const Range &range) {
    // parameters.
    int half_patch_size = 4;
    int iterations = 10;
    for(size_t i = range.start; i < range.end; ++i){
        auto kp = kp1[i];
        double dx = 0, dy = 0;
        if(has_initial){
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true;           // indicate if this point succeeded

        // Gauss-Newton iterations.
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();            // Hassian matrix.
        Eigen::Vector2d b = Eigen::Vector2d::Zero();            // bias.
        Eigen::Vector2d J;                                      // Jacobian
        for(int iter = 0; iter < iterations; iter++){
            if(inverse == false){
                H = Eigen::Matrix2d::Zero();
                b = Eigen::Vector2d::Zero();
            }else{
                // only reset b.
                b = Eigen::Vector2d::Zero();
            }
            cost = 0;
            // Compute cost and jacobians.
            for(int x = -half_patch_size; x < half_patch_size; x++)
                for(int y = -half_patch_size; y < half_patch_size; y++){
                    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) - GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);
                }
        }
    }
}