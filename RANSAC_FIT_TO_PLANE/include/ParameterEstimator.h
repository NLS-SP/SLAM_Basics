//
// Created by Robotics_qi on 2020/9/29.
//

#ifndef RANSAC_FIT_TO_PLANE_PARAMETERESTIMATOR_H
#define RANSAC_FIT_TO_PLANE_PARAMETERESTIMATOR_H

#include <vector>

template<class T, class S>
class ParameterEstimator{
public:
    /***************************************************************************************
     * @brief Extract estimation of parameters.
     * @param data The data used for estimate
     * @param parameters This vector is cleared and then filled with the computed parameters.
     ***************************************************************************************/
     virtual void estimate(std::vector<T> &data, std::vector<S> &parameters) = 0;

     /***************************************************************************************
      * Least squares estimation of parameters.
      * @param data The data used for the estimate.
      * @param parameters This vector is cleared and then filled with the computed parameters.
      **************************************************************************************/
      virtual void leastSquareEstimate(std::vector<T*> &data, std::vector<S> &parameters) = 0;

    /*************************************************************************************
     * !@brief This method tests if the given data agrees with the given model parameters.
     *************************************************************************************/
     virtual bool agree(std::vector<S>& parameters, T& data) = 0;
};

#endif //RANSAC_FIT_TO_PLANE_PARAMETERESTIMATOR_H
