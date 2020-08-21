//
// Created by Robotics_qi on 2020/8/12.
//
#include <vector>
#include <cstdlib>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

typedef pcl::PointXYZ PointType;

int main()
{
    // 读取PCD文件
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;

    pcl::io::loadPCDFile<pcl::PointXYZ>("/Users/robotics_qi/Data/kitti_dataset/lidar/pcds/000889.pcd", *point_cloud_ptr);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(point_cloud_ptr, "sample cloud1");

    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
    harris.setInputCloud(point_cloud_ptr);
    std::cout << "Input Successful" << std::endl;
    harris.setNonMaxSupression(true);
    harris.setRadius(0.06f);
    harris.setThreshold(0.02f);
    std::cout << "parameter set successful" << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>& cloud_out = *cloud_out_ptr;

    cloud_out.height = 1;
    cloud_out.width = 100;
    cloud_out.resize(cloud_out.height * cloud_out.width);
    cloud_out.clear();
    std::cout << "Extracting ....." << std::endl;
    harris.compute(cloud_out);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_harris_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& cloud_harris = *cloud_harris_ptr;
    cloud_harris.height = 1;
    cloud_harris.width = 100;
    cloud_harris.resize(cloud_out.height * cloud_out.width);
    cloud_harris.clear();

    int size = cloud_out.size();
    std::cout << "Extracting: " << size << " n keypoints." << std::endl;
    pcl::PointXYZ point;
    for(int i = 0; i < size; ++i){
        point.x = cloud_out.at(i).x;
        point.y = cloud_out.at(i).y;
        point.z = cloud_out.at(i).z;
        cloud_harris.push_back(point);
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler(cloud_harris_ptr, 255, 0, 0);
    viewer->addPointCloud(cloud_harris_ptr, harris_color_handler, "harris");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris");

    while(!viewer->wasStopped()){
        viewer->spinOnce();
    }
    return 0;
}