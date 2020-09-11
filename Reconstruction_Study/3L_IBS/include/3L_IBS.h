//
// Created by Robotics_qi on 2020/8/27.
//

#ifndef SLAM_BASICS_3L_IBS_H
#define SLAM_BASICS_3L_IBS_H
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

namespace Geometry{
    class ShapeReconstruction{
    public:
        void readPCDFile(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
        void readNormalPCDFile(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
        void setSourcePointCloud(std::vector<Eigen::Vector3d>& source_point_cloud);
        void RemoveNANAndINFData(std::vector<Eigen::Vector3d> &point_cloud_input);

        void offsetConstruction(std::vector<Eigen::Vector3d>& point_cloud, std::vector<Eigen::Vector3d>& triangulate_mesh,
                                double offset_distance);

        void BSplineModel(double size, double offset_distance);

        void viewing();
        void view_inner_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr source_point_cloud_view);
        void view_zero_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr zero_point_cloud_view);
        void view_outter_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_view);

        void reconstruct();

    protected:
        double off_set = 0.01;
        const int sampling_num_ = 100;
        std::vector<Eigen::Vector3d> data_;
        std::vector<Eigen::Vector3d> data_inner_;
        std::vector<Eigen::Vector3d> data_outter_;
        std::vector<Eigen::Vector3d> data_outter_two_;
        std::vector<Eigen::Vector3d> normal_data_;
    };
}

#endif //SLAM_BASICS_3L_IBS_H
