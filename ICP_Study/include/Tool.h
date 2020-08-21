//
// Created by Robotics_qi on 2020/8/11.
//

#ifndef BASIC_ICP_TOOL_H
#define BASIC_ICP_TOOL_H

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class TOOL{
public:
     static pcl::PointCloud<pcl::PointXYZRGB> Eigen_To_PCL(const std::vector<Eigen::Vector3d>& point_cloud, int type = 0){
         pcl::PointCloud<pcl::PointXYZRGB> point_to_view;
         pcl::PointXYZRGB temp_point;
         if(type == 0) {
             for (int i = 0; i < point_cloud.size(); ++i) {
                 temp_point.x = point_cloud[i](0);
                 temp_point.y = point_cloud[i](1);
                 temp_point.z = point_cloud[i](2);
                 temp_point.r = 0;
                 temp_point.g = 0;
                 temp_point.b = 0;
                 point_to_view.push_back(temp_point);
             }
         }else{
             for (int i = 0; i < point_cloud.size(); ++i) {
                 temp_point.x = point_cloud[i](0);
                 temp_point.y = point_cloud[i](1);
                 temp_point.z = point_cloud[i](2);
                 temp_point.r = 5;
                 temp_point.g = 10;
                 temp_point.b = 25;
                 point_to_view.push_back(temp_point);
             }
         }
         return point_to_view;
    }
};

#endif //BASIC_ICP_TOOL_H
