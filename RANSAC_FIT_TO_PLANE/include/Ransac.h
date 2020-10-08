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
    void ComputeScore();
    void EstimateModel(std::shared_ptr<T> m_BestModel, std::vector<Point3D> &data);

private:
    std::vector<Point3D> inline_data;
    std::shared_ptr<T> m_BestModel;
    std::vector<Point3D> m_BestInliers;
    int m_MaxIterations;                // Number of iterations before termination.
    float m_Threshold;                  // The threshold for computing model consensus.
    unsigned int random_seed;

};



#endif //RANSAC_LIB_H

template<typename T>
void RANSAC_Fitting<T>::EstimateModel(std::shared_ptr<T> m_BestModel, std::vector<Point3D> &data) {
    // 1. 随机选取数据作为内点
    // 1.1 先判断目前的数据能否解算出来:
    if(data.size() < 3)
        throw std::runtime_error("The data is too less to calculate.");

    // 1.2 创建随机数,将之前的数据放入内点中

    // 1.3 用内点来进行拟合和初始化模型参数

    // 2. 对模型进行打分，如果小于阈值，则返回，代表模型适用
    // 3. 如果不行，则继续迭代
}
