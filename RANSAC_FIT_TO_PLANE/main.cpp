//
// Created by Robotics_qi on 2020/9/23.
//

#include <random>
#include <vector>
#include <iostream>
#include <Geometry.h>
#include <PlaneModel.h>
#include <data_generator.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <lineParameterEstimator.h>
#include <Ransac.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main() {
    /** **************************************
     * @brief Create random sampler points with outliers to fit equation n * x + d = 0;
     * The Outliers are Gaussian Noise.
     ** *************************************/
    DataGenerator<PlaneModel, Point3D> data_generator;

    //!@brief The Plane Parameters.
    PlaneModel plane_model(0.25, 0.5, 0.75, 1.0);

    //!@brief Create 300 different points on the plane [with Gaussian Outliers].
    std::vector<Point3D> sample_points;
    data_generator.generateData(&plane_model, sample_points);
    std::cout << "Now, we have create: " << sample_points.size() << std::endl;

    /****************************************************
    * !@brief The Ransac Fitting Process.
    ****************************************************/

    PlaneModel test_svd_plane_model;
    test_svd_plane_model.parameter_estimate(sample_points);
    std::cout << "The original plane model is: " << plane_model.plane_a << ", " << plane_model.plane_b
              << ", " << plane_model.plane_c << ", " << plane_model.plane_d << std::endl;
    std::cout << "The test plane model is: " << test_svd_plane_model.plane_a << ", " << test_svd_plane_model.plane_b
              << ", " << test_svd_plane_model.plane_c << ", " << test_svd_plane_model.plane_d << std::endl;

    /********************************************************
     * !@brief The viewing window to see the test data.
     ********************************************************/
    PointCloudT::Ptr test_point_cloud(new PointCloudT);    // The test point cloud set.
    for (int i = 0; i < sample_points.size(); ++i) {
        PointT sample_point;
        sample_point.x = sample_points[i].x;
        sample_point.y = sample_points[i].y;
        sample_point.z = sample_points[i].z;
        test_point_cloud->push_back(sample_point);
    }
    std::string pcd_save_file = "/Users/robotics_qi/SLAM_Basics/RANSAC_FIT_TO_PLANE/result/plane_sample.pcd";
    pcl::io::savePCDFileASCII(pcd_save_file, *test_point_cloud);
    // Visualization
    pcl::visualization::PCLVisualizer viewer("TEST DATA");
    // Original point cloud is white
    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(test_point_cloud,
                                                                              (int) 255 * txt_gray_lvl,
                                                                              (int) 255 * txt_gray_lvl,
                                                                              (int) 255 * txt_gray_lvl);
    viewer.addPointCloud(test_point_cloud, cloud_in_color_h, "cloud_in_v1");
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

}