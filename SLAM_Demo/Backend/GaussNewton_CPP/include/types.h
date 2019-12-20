//
// Created by gatsby on 2019-02-19.
//

#ifndef PROJECT_TYPES_H
#define PROJECT_TYPES_H

#include <Eigen/Dense>
#include <functional>

namespace optimization{

    // Default precision
    typedef float Scalar;

    // Shortcut for scalar
    typedef Scalar s;

    // Default matrix type.
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

    // Default vector type.
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;

    // function prototype
    typedef std::function<Matrix(const Matrix &x)> F;

    // Computation result info
    enum resultInfo{
        SUCCESS,
        ERROR
    };

    // Dimensions of functions input and outputs.
    struct Dims{
        int x_rows;
        int x_cols;
        int y_rows;
        int y_cols;

        inline Dims(int x_rows_, int x_cols_, int y_rows_, int y_cols_)
                : x_rows(x_rows_), x_cols(x_cols_), y_rows(y_rows_), y_cols(y_cols_)
    };
}

#endif //PROJECT_TYPES_H
