//
// Created by Robotics_qi on 2020/10/12.
//

#ifndef RANSAC_FIT_TO_PLANE_DATA_GENERATOR_H
#define RANSAC_FIT_TO_PLANE_DATA_GENERATOR_H
#include <random>
#include <vector>

template<typename T, typename X>
class DataGenerator{
public:
    DataGenerator(){
        srand((unsigned)time(NULL));
    }

    void generateData(T *fitting_model, std::vector<X> &data);

protected:
    const int num_sample_data = 200;

protected:
    const double mean = 5.0;    // The Mean Value.
    const double stddev = 0.6;  // The Standard deviation.
    std::default_random_engine generator;
};

template<typename T, typename X>
void DataGenerator<T, X>::generateData(T *fitting_model, std::vector<X> &data) {
    std::normal_distribution<double> dist_1(mean, stddev);
    std::normal_distribution<double> dist_2(-4, 0.6);
    int i = 0;
    while(i++ < num_sample_data){
        X sample_point;
        sample_point.x = rand() % 100 / 5.0;
        sample_point.y = rand() % 100 / 5.0;
        sample_point.z =
                -(fitting_model->plane_d + sample_point.x * fitting_model->plane_a +
                  sample_point.y * fitting_model->plane_b) / fitting_model->plane_c + dist_1(generator);
        data.push_back(sample_point);
    }
    /*
    while(i++ < num_sample_data){
        X sample_point;
        sample_point.x = rand() % 100 / 5.0;
        sample_point.y = rand() % 100 / 5.0;
        sample_point.z =
                -(fitting_model->plane_d + sample_point.x * fitting_model->plane_a +
                  sample_point.y * fitting_model->plane_b) / fitting_model->plane_c + dist_2(generator);
        data.push_back(sample_point);
    }*/
}

#endif //RANSAC_FIT_TO_PLANE_DATA_GENERATOR_H
