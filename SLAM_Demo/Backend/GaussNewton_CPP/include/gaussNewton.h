//
// Created by gatsby on 2019-02-19.
//

#ifndef PROJECT_GAUSSNEWTON_H
#define PROJECT_GAUSSNEWTON_H

#include "types.h"

namespace optimization{

    ResultInfo gaussNewton(const F &f, const F &d, Matrix &x){
            // Make sure we have more functions than variable.
            asssert(j.rows() > j.cols());

            Matrix jt = j.transpose();
            Matrix y = f(x);

            auto llt = (jt*j).ldlt();
            if(llt.info() != Eigen::Success) return ERROR;

            Matrix s = llt.solve(jt * y * Scalar(-1));
            x = x + s;

            return SUCCESS;
    }
}
#endif //PROJECT_GAUSSNEWTON_H
