//
// Created by Robotics_qi on 2020/8/11.
//

// 这里主要是NDT的匹配和可视化
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>   // NDT 配准类头文件
#include <pcl/filters/approximate_voxel_grid.h> // 滤波类头文件（用体素格滤波器处理效果比较好）
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_pcd(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_pcd(new pcl::PointCloud<pcl::PointXYZ>());
    std::string source_cloud_path = "";
    std::string target_cloud_path = "/Users/robotics_qi/Data/kitti_dataset/lidar/pcds/000889.pcd";
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(source_cloud_path, *source_cloud_pcd) == -1){
        PCL_ERROR("Couldn't read source file from kitti dataset. ");
        return -1;
    }
    std::cout << "Loaded " << source_cloud_pcd->size() << " source data points from kitti.pcd" << std::endl;
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(target_cloud_path, *target_cloud_pcd) == -1){
        PCL_ERROR("Couldn't read target file from kitti dataset. ");
        return -1;
    }
    std::cout << "Loaded " << target_cloud_pcd->size() << " target data points from kitti.pcd" << std::endl;

    // 将输入点云降维到原始尺寸的10%提升匹配速度，对目标点云不做处理（NDT在目标点云对应的体素格网络中统计计算不适用单个点,而是用包含在每个体素格中的点的统计数据）
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(source_cloud_pcd);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filter cloud contains " << filtered_cloud->size() << " data points from kitti source pcd." << std::endl;


    // 初始化正太分布(NDT)对象
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // 依据输入数据的尺度设置NDT参数
    ndt.setTransformationRotationEpsilon(0.01);
    ndt.setStepSize(0.01);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(1000000);
    ndt.setInputSource(source_cloud_pcd);
    ndt.setInputTarget(target_cloud_pcd);
    // 使用机器人测距法得到的粗略变换矩阵结果
    Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(1, 1, 1);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ndt.align(*output_cloud, init_guess);
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()            << " score: " << ndt.getFitnessScore () << std::endl;
    pcl::transformPointCloud(*source_cloud_pcd, *output_cloud, ndt.getFinalTransformation());

    // 初始化点云可视化对象
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0, 0, 0);  //设置背景颜色为黑色
    // 对目标点云着色可视化 (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            target_color (target_cloud_pcd, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud_pcd, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    // 对转换后的源点云着色 (green)可视化.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_color (output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud"); //启动可视化
    viewer_final->addCoordinateSystem (1.0);  //显示XYZ指示轴
    viewer_final->initCameraParameters ();   //初始化摄像头参数

    // 等待直到可视化窗口关闭
    while (!viewer_final->wasStopped ())
    {
        viewer_final->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }  return (0);
}
