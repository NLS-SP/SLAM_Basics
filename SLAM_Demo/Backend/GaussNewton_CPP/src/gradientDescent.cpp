//
// Created by gatsby on 2019-02-19.
//

#include "gradientDescent.h"
#include <iostream>
#include <iomanip>

int main()
{
    // Gradient of polynom
    optimization::F df = [](const optimization::Matrix &x)->optimization::Matrix{
        optimization::Matrix d(2, 1);

        d(0) = 2.f * x(0) + 2.f;
        d(1) = 2.f * x(1) + 8.f;

        return d;
    };

    // start solution
    optimization::Matrix x(2, 1);
    x(0) = -3.f;
    x(1) = -2.f;

    // Iterate while norm of the first order derivative is greater than some predefined threshold
    optimization::ResultInfo ri = optimization::SUCCESS;
    while(ri == optimization::SUCCESS && df(x).norm() > 0.001f){
        ri = optimization::gradientDescent(df, x, 0.01f);
        std::cout << std::fixed << std::setw(3) << "Parameters: " << x.transpose() << " Error: " << df(x).norm() << std::endl;
    }

    assert(fabs(x(0) - optimization::Scalar(-1)) < optimization::Scalar(0.001));
    assert(fabs(x(1) - optimization::Scalar(-1)) < optimization::Scalar(0.001));
}