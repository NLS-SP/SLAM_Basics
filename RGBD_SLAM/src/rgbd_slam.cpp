//
// Created by Robotics_qi on 2020/8/25.
//

#include <string>
#include <iostream>

using namespace std;

// 读取OpenCV库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

// 主函数
int main()
{
    // 读取rgb和depth图像
    cv::Mat rgb, depth;
    rgb = cv::imread("");
    depth = cv::imread("", -1);

    // 点云变量
    PointCloud::Ptr cloud(new PointCloud);
    // 遍历深度图，创建点云
    for(int m = 0; m < depth.rows; ++m){
        for(int n = 0; n < depth.cols; ++n){
            ushort d = depth.ptr<ushort>(m)[n];
            // 如果d没有，则跳过此点
            if(d == 0) continue;
            // 如果存在，则增加点
            PointT p;

            // 计算空间点坐标
            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            // 从rgb图中获取颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            //将p加入点云中
            cloud->points.push_back(p);
        }
    }

    Eigen::Matrix4d transform_body_cam;
    Eigen::Matrix3d rotation_body_cam = Eigen::Matrix3d::Identity();
    Eigen::Vector4d translate_body_cam = Eigen::Vector4d(0.278, 0, 0, 1);
    transform_body_cam.block<3, 3>(0, 0) = rotation_body_cam;
    transform_body_cam.rightCols<1>() = translate_body_cam;


    for(auto point : *cloud){
        // 点云点齐次坐标向量化：
        Eigen::Vector4d point_cam_coordinate(point.x, point.y, point.z, 1);
        // 转换到body下:
        Eigen::Vector4d point_body_coordinate = transform_body_cam * point_cam_coordinate;
        // TODO: 转到世界系下：
        // transform_body_cam.block<3, 3>(0, 0) = R_wb;
        // transform_body_cam.rightCols<1>() = t_w;(注意齐次坐标向量化)
        // Eigen::Vector4d point_world_coordinate = transform_world_body * point_body_coordinate;
    }
}