//
// Created by Robotics_qi on 2021/1/4.
//
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main()
{
    Eigen::Matrix4d received_pose = Eigen::Matrix4d::Identity();
    received_pose(0,3) = 0.1;
    received_pose(1,3) = 0.2;
    received_pose(2,3) = 0.3;
    std::cout << received_pose << std::endl;
    Eigen::Matrix4d idea_pose = Eigen::Matrix4d::Zero();
    idea_pose(0, 0) =  1.0;
    idea_pose(1, 2) =  1.0;
    idea_pose(2, 1) = -1.0;
    idea_pose(3, 3) =  1.0;
    Eigen::Matrix4d transform_kitti;
    transform_kitti = idea_pose * received_pose.inverse();
    Eigen::Matrix4d transformed_pose;
    transformed_pose = transform_kitti * received_pose;
    std::cout << "The transformed pose is: " << std::endl << transformed_pose << std::endl;
}