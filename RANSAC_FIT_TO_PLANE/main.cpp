//
// Created by Robotics_qi on 2020/9/23.
//

#include <iostream>
#include <vector>
#include <Geometry.h>
#include <PlaneModel.h>
#include <ParameterEstimator.h>
#include <lineParameterEstimator.h>

int main()
{
    // 1. Create data with outliers.
    // Randomly select a direction [dx, dy] and create a line passing through the origin.
    // For each point sampled on the line and add random noise, finally add outlying points in the direction of the line normal.
    srand((unsigned)time(NULL));    // seed random number generator.


}