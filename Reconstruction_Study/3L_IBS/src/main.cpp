//
// Created by Robotics_qi on 2020/8/27.
//

#include <3L_IBS.h>
#include <pcl/io/pcd_io.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_pcd(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_normal_cloud_pcd(new pcl::PointCloud<pcl::PointXYZ>());
    std::string source_cloud_path = "/Users/robotics_qi/SLAM_Basics/Reconstruction_Study/3L_IBS/Data/Duck.pcd";
    std::string source_cloud_normal_path = "/Users/robotics_qi/SLAM_Basics/Reconstruction_Study/3L_IBS/Data/Duck_Norm.pcd";
    pcl::io::loadPCDFile(source_cloud_path, *source_cloud_pcd);
    pcl::io::loadPCDFile(source_cloud_normal_path, *source_normal_cloud_pcd);
    Geometry::ShapeReconstruction shape_modeling;
    shape_modeling.readPCDFile(source_cloud_pcd);
    shape_modeling.readNormalPCDFile(source_normal_cloud_pcd);
    shape_modeling.reconstruct();
//    shape_modeling.viewing();
}