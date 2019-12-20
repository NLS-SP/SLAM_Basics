//
// Created by Robotics_qi on 2019-09-02.
//

#ifndef SLAM_FRONTED_RANSAC_H
#define SLAM_FRONTED_RANSAC_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

class RansacOptions{
public:
    RansacOptions():
        min_num_iterations_(100u), max_num_iterations_(1000u),
        success_probability_(0.9999), squared_inlier_threshold_(1.0),
        random_seed_(0u){}

    uint32_t min_num_iterations_;
    uint32_t max_num_iterations_;
    double success_probability_;
    double squared_inlier_threshold_;
    unsigned int random_seed_;
};

class LORansacOptions : public RansacOptions{
public:
    LORansacOptions():
        num_lo_steps_(10), threshold_multiplier_(std::sqrt(2.0)),
        num_lsq_iterations_(4), min_sample_multiplicator_(7),
        non_min_sample_multiplier_(3), lo_starting_iterations(50u),
        final_least_squares_(false){}

    int num_lo_steps_;
    double threshold_multiplier_;
    int num_lsq_iterations_;
    int min_sample_multiplicator_;
    int non_min_sample_multiplier_;

    uint32_t lo_starting_iterations_;
    bool final_least_squares_;
};


#endif //SLAM_FRONTED_RANSAC_H
