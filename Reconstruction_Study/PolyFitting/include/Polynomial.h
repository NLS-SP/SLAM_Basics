//
// Created by Robotics_qi on 2020/8/30.
//

#ifndef INC_3L_IBS_POLYNOMIAL_H
#define INC_3L_IBS_POLYNOMIAL_H

#include <cmath>
#include <vector>
#include <string>
#include <cstdlib>
#include <iostream>

namespace Geometry{
    class Polynomial{
    public:
    protected:
        int dims;

        int order;
        int numBasis;
        std::vector<double> powers;
        std::vector<double> coefficients;
    };
}

#endif //INC_3L_IBS_POLYNOMIAL_H
