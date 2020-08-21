//
// Created by Robotics_qi on 2020/8/10.
//

#include <iostream>
#include <Tool.h>
#include <thread>
#include <Registration.h>


int main()
{
    // 读取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_pcd(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_pcd(new pcl::PointCloud<pcl::PointXYZ>());
    std::string source_cloud_path = "/Users/robotics_qi/SLAM_Basics/ICP_Study/Data/bunny_RRBR_Source.pcd";
    std::string model_cloud_path = "/Users/robotics_qi/SLAM_Basics/ICP_Study/Data/bunny_RRBR_Model.pcd";
    pcl::io::loadPCDFile(source_cloud_path, *source_cloud_pcd);
    pcl::io::loadPCDFile(model_cloud_path, *target_cloud_pcd);

    MSP_REGISTRATION_METHOD ICP_Register;
    ICP_Register.setSourcePointCloud(*source_cloud_pcd);
    ICP_Register.setTargetPointCloud(*target_cloud_pcd);

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_point_cloud_view(new pcl::PointCloud<pcl::PointXYZ>());
    ICP_Register.view_source_point_cloud(source_point_cloud_view);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_view(new pcl::PointCloud<pcl::PointXYZ>());
    ICP_Register.view_target_point_cloud(target_point_cloud_view);

    // 这里是可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ICP Viewer"));
    // 设置背景颜色
    viewer->setBackgroundColor(0, 0, 0);
    // 添加坐标系,这里参数是坐标轴的长度
    viewer->addCoordinateSystem(0.2);
    // 对source点云着色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source_point_cloud_view, 255, 0, 0);
    viewer->addPointCloud(source_point_cloud_view, source_color, "source cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
    // 将Target Point信息可视化先给关闭
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_point_cloud_view, 0, 255, 0);
    viewer->addPointCloud(target_point_cloud_view, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

    viewer->initCameraParameters();
    while(!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}