//
// Created by Robotics_qi on 2020/2/15.
//

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// A cost functor that implements the residual r = 10 - x;
struct CostFunctor{
    bool operator()(const double* const x, double* residual) const{
        residual[0] = 10.0 - x[0];
        return true;
    }
};

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    double x = 0.5;
    const double initial_x = x;

    // Build the problem.
    Problem problem;

    // Set up the only cost function(residual)
    CostFunction* cost_function = new NumericDiffCostFunction<CostFunctor, CENTRAL, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the Solver.
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x: " << initial_x << " -> " << x << "\n";
    return 0;
}