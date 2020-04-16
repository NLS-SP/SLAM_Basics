//
// Created by Robotics_qi on 2020/2/15.
//

#include <vector>
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class QuadraticCostFunction : public SizedCostFunction<1, 1>{
public:
    virtual ~QuadraticCostFunction(){}

    virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const{
        double x = parameters[0][0];

        // f(x) = 10 - x
        residuals[0] = 10 - x;

        if(jacobians != NULL && jacobians[0] != NULL){
            jacobians[0][0] = -1;
        }
        return true;
    }
};

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    double x = 0.5;
    const double initial_x = 0.5;

    // build the problem.
    Problem problem;

    CostFunction* cost_function = new QuadraticCostFunction;
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver.
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x: " << initial_x << " -> " << x << std::endl;
    return 0;
}