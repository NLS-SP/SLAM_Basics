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

// Optical Flow Tracker and Interface.
class OpticalFlowTracker{
public:
    OpticalFlowTracker(const cv::Mat& image1_, const cv::Mat& image2_,
                       const std::vector<cv::KeyPoint>& kp1_,
                       std::vector<cv::KeyPoint>& kp2_, std::vector<bool>& success_,
                       bool inverse_ = true, bool has_initial_ = false):
                       image_1(image1_), image_2(image2_), kp1(kp1_), kp2(kp2_), success(success_), inverse(inverse_),
                       has_initial(has_initial_){}

    void calculateOpticalFlow(const Range& range);

private:
    const cv::Mat& image_1;
    const cv::Mat& image_2;
    const std::vector<cv::KeyPoint>& kp1;
    std::vector<cv::KeyPoint>& kp2;
    std::vector<bool>& success;
    bool inverse = true;
    bool has_initial = false;
};

void OpticalFlowSingleLevel(
        const cv::Mat& image_1,
        const cv::Mat& image_2,
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2,
        std::vector<bool> success,
        bool inverse = false,
        bool has_initial_guess = false);

int main(int argc, char **argv)
{
    std::string file_1 = "";
    std::string file_2 = "";
    cv::Mat image_1 = cv::imread(file_1, 0);
    cv::Mat image_2 = cv::imread(file_2, 0);

    // Detect Key Points, using GFTT.
    std::vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20);

    // Track the keypoint in the second image.
    // first use single level LK in the validation picture.
    std::vector<cv::KeyPoint> kp2_single;
    std::vector<bool> success_single;
    OpticalFlowSingleLevel(image_1, image_2, kp1, kp2_single, success_single);
}

void OpticalFlowSingleLevel(const cv::Mat& image_1, const cv::Mat& image_2, const std::vector<cv::KeyPoint>& kp1,
                        std::vector<cv::KeyPoint>& kp2, std::vector<bool>& success, bool inverse, bool has_initial){
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    OpticalFlowTracker tracker(image_1, image_2, kp1, kp2, success, inverse, has_initial);

}

void OpticalFlowTracker::calculateOpticalFlow(const Range &range) {
    // parameters.
    int half_patch_size = 4;
    int iterations = 10;
    for(size_t i = range.start; i < range.end; i++){
        auto kp = kp1[i];
        double dx = 0, dy = 0;
        if(has_initial){
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }
        double cost = 0, lastCost = 0;
        bool succ = true;       // indicate if this point succeeded.

        // Gauss-Newton iterations.
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();    // hessian.
        Eigen::Vector2d b = Eigen::Vector2d::Zero();    // bias.
        Eigen::Vector2d J;                              // Jacobian.
        for(int iter = 0; iter < iterations; iter++){
            if(inverse == false){
                H = Eigen::Matrix2d::Zero();
                b = Eigen::Vector2d::Zero();
            }else{
                // Only Reset b.
                b = Eigen::Vector2d::Zero();
            }

            cost = 0;

            // Compute the cost and jacobians.

            // Compute update.
            Eigen::Vector2d update = H.ldlt().solve(b);
            if(std::isnan(update[0])){
                std::cout << "" << std::endl;
                succ = false;
                break;
            }
            if(iter > 0 && cost > lastCost)
                break;

            // update dx, dy.
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;

            if(update.norm() < 1e - 2) break;

            success[i] = succ;

            // set kp2. -> 这里实际上是做好第一幅图像和第二幅图像的关联地方
            kp2[i].pt = kp.pt + cv::Point2f(dx, dy);
            // 能不能这样，计算出光流法：这个特征点在下幅图像中对应的位置，然后反推找出直线的位置，利用像素来。
        }
    }
}