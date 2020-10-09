//
// Created by Robotics_qi on 2020/9/23.
//

#include <random>
#include <vector>
#include <iostream>
#include <Geometry.h>
#include <PlaneModel.h>
#include <ParameterEstimator.h>
#include <lineParameterEstimator.h>
#include <Ransac.h>

int main()
{
    /** **************************************
     * @brief Create random sampler points with outliers to fit equation n * x + d = 0;
     * The Outliers are Gaussian Noise.
     ** *************************************/
    srand((unsigned)time(NULL));

    //!@brief The Plane Parameters.
    double plane_parameter[4] = {1.0, 2.0, 3.0, 4.0};

    //!@brief The Gaussian Outliers.
    const double mean = 0.0;    // The Mean Value.
    const double stddev = 0.1;  // The Standard deviation.
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);


    //!@brief Create 300 different points on the plane [with Gaussian Outliers].
    std::vector<Point3D> sample_points;
    for(int i = 0; i < 300; i ++) {
        Point3D sample_point;
        sample_point.x = rand() % 100 / 5.0;
        sample_point.y = rand() % 100 / 5.0;
        sample_point.z =
                -(plane_parameter[3] + sample_point.x * plane_parameter[0] + sample_point.y * plane_parameter[1]) /
                plane_parameter[2] + dist(generator);
        sample_points.push_back(sample_point);
    }

    /****************************************************
     * !@brief The Ransac Fitting Process.
     ****************************************************/
    PlaneModel plane_model;
    RANSAC_Fitting<PlaneModel> ransac_fitting;
    for(int i = 0; i < 10; ++i)
        ransac_fitting.Roubust_Fitting(&plane_model, sample_points);

}