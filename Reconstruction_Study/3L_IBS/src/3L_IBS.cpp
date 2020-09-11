//
// Created by Robotics_qi on 2020/8/27.
//

#include <3L_IBS.h>
#include <streambuf>

namespace Geometry {
    void ShapeReconstruction::readPCDFile(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
        std::vector<Eigen::Vector3d> source_point_cloud_eigen;
        for (int i = 0; i < (*point_cloud).size(); ++i)
            source_point_cloud_eigen.push_back(
                    Eigen::Vector3d((*point_cloud)[i].x, (*point_cloud)[i].y, (*point_cloud)[i].z));

        setSourcePointCloud(source_point_cloud_eigen);
    }

    void ShapeReconstruction::readNormalPCDFile(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
        std::vector<Eigen::Vector3d> normal_point_cloud_eigen;
        for (int i = 0; i < (*point_cloud).size(); ++i)
            normal_point_cloud_eigen.push_back(
                    Eigen::Vector3d((*point_cloud)[i].x, (*point_cloud)[i].y, (*point_cloud)[i].z));
        normal_data_ = normal_point_cloud_eigen;
    }

    void ShapeReconstruction::setSourcePointCloud(std::vector<Eigen::Vector3d> &source_point_cloud) {
        data_ = source_point_cloud;
        RemoveNANAndINFData(data_);
        std::cout << "Now, we have " << data_.size() << " points " << std::endl;
    }

    void ShapeReconstruction::RemoveNANAndINFData(std::vector<Eigen::Vector3d> &point_cloud_input) {
        // 非法数据去除
        for (std::vector<Eigen::Vector3d>::iterator it = point_cloud_input.begin(); it != point_cloud_input.end();) {
            Eigen::Vector3d temp_point_cloud = *it;
            if (std::isnan(temp_point_cloud(0)) || std::isnan(temp_point_cloud(1)) || std::isnan(temp_point_cloud(2))
                || std::isinf(temp_point_cloud(0)) || std::isinf(temp_point_cloud(1)) ||
                std::isinf(temp_point_cloud(2)))
                it = point_cloud_input.erase(it);
            else
                it++;
        }
    }

    void ShapeReconstruction::reconstruct() {
        // P = zeros(N, N, N);
        std::vector<Eigen::MatrixXd> P;
        for (int i = 0; i < sampling_num_; ++i)
            P.push_back(Eigen::MatrixXd::Zero(sampling_num_, sampling_num_));

        //!@brief The Matrix M.
        std::vector<Eigen::Vector3d> M;

        //!@brief Caculate offset
        for (int i = 0; i < data_.size(); ++i) {
            data_outter_.push_back(Eigen::Vector3d(off_set * normal_data_[i] + data_[i]));
            data_outter_two_.push_back(Eigen::Vector3d(2 * off_set * normal_data_[i] + data_[i]));
            data_inner_.push_back(Eigen::Vector3d(data_[i] - 2 * off_set * normal_data_[i]));
        }
        M = data_;
        for (int i = 0; i < data_outter_.size(); ++i)
            M.push_back(data_outter_[i]);
        for (int i = 0; i < data_inner_.size(); ++i)
            M.push_back(data_inner_[i]);
        int n = M.size(), P_M = 100, P_N = 100, P_O = 100;
        std::vector<Eigen::MatrixXd> IND_Vector;
        for (int i = 0; i < sampling_num_; ++i)
            IND_Vector.push_back(Eigen::MatrixXd::Zero(sampling_num_, sampling_num_));

        double step_x = 1.0 / (P_M - 3);
        double step_y = 1.0 / (P_N - 3);
        double step_k = 1.0 / (P_O - 3);
        std::vector<int> i_vector, j_vector, k_vector;
        i_vector.clear();
        j_vector.clear();
        k_vector.clear();

        for (int i = 0; i < M.size(); i++) {
            i_vector.push_back(std::floor(M[i].x() / step_x) + 1);
            j_vector.push_back(std::floor(M[i].y() / step_y) + 1);
            k_vector.push_back(std::floor(M[i].z() / step_k) + 1);
        }
        std::cout << "Start to calculate the index....." << std::endl;
        for (int h = 0; h < M.size(); h++) {
            for (int i_h = 0; i_h < 4; i_h++) {
                for (int j_h = 0; j_h < 4; j_h++) {
                    for (int k_h = 0; k_h < 4; k_h++) {
                        IND_Vector[i_vector[h] + i_h](j_vector[h] + j_h, k_vector[h] + k_h) = 1;
                    }
                }
            }
        }

        int count = 1;
        for (int i = 0; i < IND_Vector.size(); i++) {
            for (int j = 0; j < IND_Vector[0].rows(); j++) {
                for (int k = 0; k < IND_Vector[0].cols(); k++) {
                    if (IND_Vector[i](j, k) == 1) IND_Vector[i](j, k) = count++;// 这里开始值有偏差
                }
            }
        }
        std::cout << "Finish!" << std::endl;
        std::cout << count << std::endl;
    }

    void ShapeReconstruction::BSplineModel(double size, double offset_distance) {

    }

    void ShapeReconstruction::offsetConstruction(std::vector<Eigen::Vector3d>& point_cloud, std::vector<Eigen::Vector3d>& triangulate_mesh,
                                                 double offset_distance) {

    }

    void ShapeReconstruction::viewing() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr inner_point_cloud_view(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr zero_point_cloud_view(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr outter_point_cloud_view(new pcl::PointCloud<pcl::PointXYZ>());

        view_inner_point_cloud(inner_point_cloud_view);
        view_zero_point_cloud(zero_point_cloud_view);
        view_outter_point_cloud(outter_point_cloud_view);

        // 这里是可视化
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ICP Viewer"));
        // 设置背景颜色
        viewer->setBackgroundColor(0, 0, 0);
        // 添加坐标系,这里参数是坐标轴的长度
        viewer->addCoordinateSystem(0.2);
        // 对source点云着色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> zero_color(zero_point_cloud_view, 255, 0, 0);
        viewer->addPointCloud(zero_point_cloud_view, zero_color, "zero cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "zero cloud");
        // 将Target Point信息可视化先给关闭
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inner_color(inner_point_cloud_view, 0, 255, 0);
        viewer->addPointCloud(inner_point_cloud_view, inner_color, "inner cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "inner cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outter_color(outter_point_cloud_view, 0, 0, 255);
        viewer->addPointCloud(outter_point_cloud_view, outter_color, "two offset cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "two offset cloud");

        viewer->initCameraParameters();
        while(!viewer->wasStopped()) {
            viewer->spinOnce(100);
        }

    }

    void ShapeReconstruction::view_inner_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inner_point_cloud_view) {
        pcl::PointXYZ temp_point;
        if(data_inner_.size() == 0) std::cout << "Haven't read any cloud!" << std::endl;
        for (int i = 0; i < data_inner_.size(); ++i) {
            temp_point.x = data_inner_[i](0);
            temp_point.y = data_inner_[i](1);
            temp_point.z = data_inner_[i](2);
            inner_point_cloud_view->push_back(temp_point);
        }
    }
    void ShapeReconstruction::view_zero_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inner_point_cloud_view) {
        pcl::PointXYZ temp_point;
        if(data_inner_.size() == 0) std::cout << "Haven't read any cloud!" << std::endl;
        for (int i = 0; i < data_inner_.size(); ++i) {
            temp_point.x = data_[i](0);
            temp_point.y = data_[i](1);
            temp_point.z = data_[i](2);
            inner_point_cloud_view->push_back(temp_point);
        }
    }

    void ShapeReconstruction::view_outter_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr outter_point_cloud_view) {
        pcl::PointXYZ temp_point;
        if(data_outter_.size() == 0) std::cout << "Haven't read any cloud!" << std::endl;
        for (int i = 0; i < data_outter_.size(); ++i) {
            temp_point.x = data_outter_two_[i](0);
            temp_point.y = data_outter_two_[i](1);
            temp_point.z = data_outter_two_[i](2);
            outter_point_cloud_view->push_back(temp_point);
        }
    }
}