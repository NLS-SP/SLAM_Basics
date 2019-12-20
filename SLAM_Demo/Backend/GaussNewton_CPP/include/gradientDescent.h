//
// Created by gatsby on 2019-02-19.
//

#ifndef PROJECT_GRADIENTDESCENT_H
#define PROJECT_GRADIENTDESCENT_H

#include "types.h"

namespace optimization{
    resultInfo gradientDescent(const F &d, Matrix &x, Scalar step){
        x = x - step * d(x);
        return SUCCESS;
    }
}

#endif //PROJECT_GRADIENTDESCENT_H
