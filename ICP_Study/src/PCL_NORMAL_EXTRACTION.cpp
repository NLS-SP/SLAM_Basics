//
// Created by Robotics_qi on 2020/8/12.
//

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    // 点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_pcd(new pcl::PointCloud<pcl::PointXYZ>());
    std::string source_cloud_path = "/Users/robotics_qi/Data/livox_dataset/bin/indoor_8/67.pcd";
    pcl::io::loadPCDFile(source_cloud_path, *source_cloud_pcd);
    // 法线估计向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(source_cloud_pcd);
    // 创建空的KDTree对象，并把它传递给法线估计向量
    pcl::seasrch::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    normal_estimation.setSearchMethod(tree);
    // 存储输出数据
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // 使用半径查询点周围5m范围内的所有临近元素
    normal_estimation.setRadiusSearch(5);
    // 计算特征值
    normal_estimation.compute(*cloud_normals);
    // 可视化
    pcl::visualization::PCLVisualizer viewer("PCL NORMAL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(source_cloud_pcd, cloud_normals);
    while(!viewer.wasStopped()){
        viewer.spinOnce();
    }
    return 0;
}