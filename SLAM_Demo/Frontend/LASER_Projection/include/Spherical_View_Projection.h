//
// Created by Robotics_qi on 2020/7/16.
//

#ifndef SLAM_FRONTED_SPHERICAL_VIEW_PROJECTION_H
#define SLAM_FRONTED_SPHERICAL_VIEW_PROJECTION_H

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct Configuration{
    double fov_up = 0.0;
    double fov_down = 0.0;
    double num_lasers = 0.0;
    double img_length = 0.0;
};

class SphericalConversion{
public:

    SphericalConversion(const Configuration& config);

    int loadCloud(const std::string& path);

    int MakeImage();

    void GetProjection(const pcl::PointXYZI& point, const double& fov_rad, const double& fov_down_rad, int* pixel_x, int* pixel_y, double *depth) const;

    std::vector<std::vector<std::vector<double>>> GetImage() const;

    void ShowImg(const std::vector<std::vector<std::vector<double>>>& img) const;
private:

    const Configuration config_;

    std::vector<std::vector<std::vector<double>>> spherical_img_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;

};


#endif //SLAM_FRONTED_SPHERICAL_VIEW_PROJECTION_H
