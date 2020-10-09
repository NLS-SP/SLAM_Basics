//
// Created by Robotics_qi on 2020/9/23.
//

#ifndef RANSAC_LIB_H
#define RANSAC_LIB_H

#include <memory>
#include <vector>
#include <Geometry.h>

template<class T>
class RANSAC_Fitting {
public:
    RANSAC_Fitting();

    T* GetBestModel();

    double ComputeScore(T *plane_model, std::vector<Point3D> &score_points);

    void EstimateModel(T *plane_model, std::vector<Point3D> &data);

    void Roubust_Fitting(T *plane_model, std::vector<Point3D> &data);

private:
    std::shared_ptr<T> m_BestModel;
    std::vector<Point3D> m_BestInliers;
    int m_MaxIterations;                // Number of iterations before termination.[If iteration is less, It will cause the local optimize problem.]
    float m_Threshold;                  // The threshold for computing model consensus.
    unsigned int random_seed;
    float m_best_score;
};

#endif //RANSAC_LIB_H



template<typename T>
RANSAC_Fitting<T>::RANSAC_Fitting() {
    m_BestModel = nullptr;
    m_BestInliers.clear();
    m_MaxIterations = 100;// std::numeric_limits<double>::max();
    m_Threshold = 0.1;
    m_best_score = std::numeric_limits<double>::min();
    srand((unsigned)time(NULL));    // The Random seed.
}

template<typename T>
double RANSAC_Fitting<T>::ComputeScore(T *plane_model, std::vector<Point3D> &score_points) {
    double sum = 0;
    for (int i = 0; i < score_points.size(); ++i) {
        double distance = plane_model->plane_a * score_points[i].x + plane_model->plane_b * score_points[i].y +
                          plane_model->plane_c * score_points[i].z + plane_model->plane_d;
        sum += distance;
    }
    sum = sum / score_points.size();
    score_points.clear();
    return sum;
}

template<typename T>
void RANSAC_Fitting<T>::EstimateModel(T *plane_model, std::vector<Point3D> &data) {
    double a, b, c, d;
    double x0 = data[0].x;
    double y0 = data[0].y;
    double z0 = data[0].z;
    double x1 = data[1].x;
    double y1 = data[1].y;
    double z1 = data[1].z;
    double x2 = data[2].x;
    double y2 = data[2].y;
    double z2 = data[2].z;
    double x3 = data[3].x;
    double y3 = data[3].y;
    double z3 = data[3].z;
}

template<typename T>
T* RANSAC_Fitting<T>::GetBestModel(){
    return m_BestModel;
}

template<typename T>
void RANSAC_Fitting<T>::Roubust_Fitting(T* plane_model, std::vector<Point3D> &data) {
    // 1. 随机选取数据作为内点
    // 1.1 先判断目前的数据能否解算出来:
    if (data.size() < 4)
        throw std::runtime_error("The data is too less to calculate.");

    // 1.2 创建随机数,将之前的数据放入内点中
    int iteration = 0;

    while(iteration++ < m_MaxIterations) {
        double random_variable[4] = {0, 0, 0, 0};
        std::vector<Point3D> fitting_points;
        random_variable[0] = rand() % 300;
        random_variable[1] = rand() % 300;
        random_variable[2] = rand() % 300;
        random_variable[3] = rand() % 300;
        // TODO: 目前这里是4D计算，比较low，实际上应该多点计算
        fitting_points.push_back(data[random_variable[0]]);
        fitting_points.push_back(data[random_variable[1]]);
        fitting_points.push_back(data[random_variable[2]]);
        fitting_points.push_back(data[random_variable[3]]);

        std::vector<Point3D> score_points;
        for(int i = 0; i < 100; i++) {
            int select = rand() % 300;
            if (select == random_variable[0] || select == random_variable[1] || select == random_variable[2] ||
                select == random_variable[3])
                continue;
            else
                score_points.push_back(data[select]);
        }
        std::cout << "The number of calculating points: " << score_points.size() << std::endl;

        // 1.3 用内点来进行拟合和初始化模型参数
        EstimateModel(plane_model, fitting_points);

        // 2. 对模型进行打分，如果小于阈值，则返回，代表模型适用
        double score_point = ComputeScore(plane_model, score_points);
        if(score_point > m_best_score) {
            m_best_score = score_point;
            m_BestModel = std::shared_ptr<T>(plane_model);
            m_BestInliers = fitting_points;
            if(score_point < m_Threshold)
                break;
        }

    }
    std::cout << "Couldn't find model in the threshold, after iterating " << iteration << " times, the plane model is: "
              << plane_model << std::endl;
    return;

    // 3. 如果不行，则继续迭代
}
