//
// Created by Robotics_qi on 2020/7/16.
//

#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Spherical_View_Projection.h>
#include <opencv2/core/mat.hpp>
#include <opencv/cv.hpp>

SphericalConversion::SphericalConversion(const Configuration& config):
                     config_(config){
    spherical_img_.assign(config_.num_lasers, std::vector<std::vector<double>>(config_.img_length, std::vector<double>(5, 0.0)));
    cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

int SphericalConversion::loadCloud(const std::string& path){
    // loading from Bin File.
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud_) == -1){
        std::cout << "Couldn't read cloud file at: " << path << std::endl;
        return -1;
    }
    return 1;
}

int SphericalConversion::MakeImage(){
    // 首先转换成弧度
    double fov_up_rad = (config_.fov_up / 180) * M_PI;
    double fov_down_rad = (config_.fov_down / 180) * M_PI;
    // Getting total field of view.
    double fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);
    if(cloud_->size() == 0){
        std::cerr << "Empty Point Cloud." << std::endl;
        return -1;
    }
    for(auto point : *cloud_) {
        // Getting Pixel from Point.
        int pixel_u = 0;
        int pixel_v = 0;
        double range = 0.0;
        GetProjection(point, fov_rad, fov_down_rad, &pixel_v, &pixel_u, &range);
        spherical_img_.at(pixel_u).at(pixel_v) = std::vector<double>{point.x, point.y, point.z, range, point.intensity};
    }
    return 1;
}

void SphericalConversion::GetProjection(const pcl::PointXYZI& point, const double& fov_rad, const double& fov_down_rad, int* pixel_v, int* pixel_u,
                                       double* range) const{
    *range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    // Geting the angle of all points.
    auto yaw = atan2(point.y, point.x);
    auto pitch = asin(point.z / *range);
    // Get projection in image coords and normalizing
    // Basic Implementation
    double v = 0.5 * (1.0 - yaw / M_PI);
    double u = 1.0 - (pitch + std::abs(fov_down_rad)) / fov_rad;
    // Scaling as per the lidar config given.
    v *= config_.img_length;
    u *= config_.num_lasers;
    // round and clamp for use as index.
    v = floor(v);
    v = std::min(config_.img_length - 1, v);
    v = std::max(0.0, v);
    *pixel_v = int(v);

    u = floor(u);
    u = std::min(config_.num_lasers - 1, u);
    u = std::max(0.0, u);
    *pixel_u = int(u);
}

std::vector<std::vector<std::vector<double>>> SphericalConversion::GetImage() const { return spherical_img_; }

void SphericalConversion::ShowImg(const std::vector<std::vector<std::vector<double>>>& img) const{
    cv::Mat sp_img(img.size(), img.at(0).size(), CV_64FC1);
    for(int i = 0; i < sp_img.rows; ++i)
        for(int j = 0; j < sp_img.cols; ++j)
            sp_img.at<double>(i, j) = img.at(i).at(j).at(4);

    cv::imshow("Intensity Image", sp_img);
    cv::waitKey(0);
}

