//
// Created by Robotics_qi on 2020/8/10.
//

#ifndef BASIC_ICP_REGISTRATION_H
#define BASIC_ICP_REGISTRATION_H


#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/EigenValues>
#include <unsupported/Eigen/Polynomials>
// 基于PCL的文件读取
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// 基于PCL的点云类型选择
#include <pcl/point_types.h>
// 基于PCL的点云特征抽取
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <Tool.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <nabo/nabo.h>

enum POINT_FEATURES{CURVATURE, PFH, FPFH};
enum REGISTRATION_METHOD{ BASIC_ICP, GO_ICP, IMLS_ICP_Polynomials, IMLS_ICP_BSplines};

class MSP_REGISTRATION_METHOD{
public:
    MSP_REGISTRATION_METHOD(REGISTRATION_METHOD registration_method = BASIC_ICP);
    ~MSP_REGISTRATION_METHOD();

    void setIteration(uint32_t iteration);

    void setSourcePointCloud(std::vector<Eigen::Vector3d>& source_point_cloud);

    void setSourcePointCloud(pcl::PointCloud<pcl::PointXYZ>& source_point_cloud_pcl);

    void setSourcePointCloudNormals(std::vector<Eigen::Vector3d>& normals);

    void setTargetPointCloudNormals(std::vector<Eigen::Vector3d>& normals);

    void setTargetPointCloud(std::vector<Eigen::Vector3d>& target_point_cloud);

    void setTargetPointCloud(pcl::PointCloud<pcl::PointXYZ>& target_point_cloud_pcl);

    void internal_external_offset_compute();

    void view_source_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr source_point_cloud_view);
    void view_target_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_view);


protected:
    REGISTRATION_METHOD registration_method_;

    uint32_t iter_;

    void RemoveNANAndINFData(std::vector<Eigen::Vector3d>& point_cloud_input);

    // 目标点云和当前点云，目标点云为参考点云
    std::vector<Eigen::Vector3d> current_source_point_cloud_, current_target_point_cloud_;

    // 目标点云和当前点云的法向量
    std::vector<Eigen::Vector3d> current_source_point_cloud_normals, current_target_point_cloud_normals;

    // 激光帧数据和对应
    std::map<int, std::vector<int> > m_laser_frames;

    // 树指针
    Nabo::NNSearchD* target_KDTree_pointer;
    Nabo::NNSearchD* source_KDTree_pointer;

    // 数据库
    Eigen::MatrixXd source_KDTree_DataBase;
    Eigen::MatrixXd target_KDTree_DataBase;

    // 迭代次数
    int iterations_;

    // 点云id
    int point_id;

    bool normals_valid_;

};


#endif //BASIC_ICP_REGISTRATION_H
