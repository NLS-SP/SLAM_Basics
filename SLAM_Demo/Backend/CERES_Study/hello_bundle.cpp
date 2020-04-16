//
// Created by Robotics_qi on 2020/2/17.
//

#include <cmath>
#include <cstdio>
#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

// Read a bundle adjustment in the large dataset
class BALProblem{
public:
    ~BALProblem(){
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }

    int num_observations() const{ return num_observations_; }
    const double* observations() const { return observations_; }
    double* mutable_cameras() { return parameters_; }
    double* mutable_points() { return parameters_ + 9 * num_cameras_; }
    double* mutable_camera_for_observation(int i){
        return mutable_cameras() + camera_index_[i] * 9;
    }

    double* mutable_points_for_observation(int i){
        return mutable_points() + point_index_[i] * 3;
    }

    bool LoadFile(const char* filename) {
        FILE* fptr = fopen(filename, "r");
        if (fptr == NULL) {
            return false;
        };
        FscanfOrDie(fptr, "%d", &num_cameras_);
        FscanfOrDie(fptr, "%d", &num_points_);
        FscanfOrDie(fptr, "%d", &num_observations_);
        point_index_ = new int[num_observations_];
        camera_index_ = new int[num_observations_];
        observations_ = new double[2 * num_observations_];
        num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
        parameters_ = new double[num_parameters_];
        for (int i = 0; i < num_observations_; ++i) {
            FscanfOrDie(fptr, "%d", camera_index_ + i);
            FscanfOrDie(fptr, "%d", point_index_ + i);
            for (int j = 0; j < 2; ++j) {
                FscanfOrDie(fptr, "%lf", observations_ + 2*i + j);
            }
        }
        for (int i = 0; i < num_parameters_; ++i) {
            FscanfOrDie(fptr, "%lf", parameters_ + i);
        }
        return true;
    }

private:
    template<typename T>
    void FscanfOrDie(FILE *fptr, const char *format, T* value){
        int num_scanned = fscanf(fptr, format, value);
        if(num_scanned != 1)
            LOG(FATAL) << "Invalid UW data file.";
    }

    int num_cameras_;
    int num_points_;
    int num_observations_;
    int num_parameters_;

    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* parameters_;
};

struct snavely_reprojection_error{
    snavely_reprojection_error(double observed_x, double observed_y):
        observed_x(observed_x), observed_y(observed_y){}

    template<typename T>
    bool operator()(const T* const camera, const T* const point, T* residual) const{
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);

        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        T xp = - p[0] / p[2];
        T yp = - p[1] / p[2];

        const T& l1 = camera[7];
        const T& l2 = camera[8];
        T r2 = xp*xp + yp*yp;
        T distortion = 1.0 + r2 * (l1 + l2 * r2);

        // Compute final projected point position.
        const T& focal = camera[6];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;

        residual[0] = predicted_x - observed_x;
        residual[1] = predicted_y - observed_y;

        return true;
    }

    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y){
        return(new ceres::AutoDiffCostFunction<snavely_reprojection_error, 2, 9, 3>(new snavely_reprojection_error(observed_x, observed_y)));
    }

    double observed_x;
    double observed_y;
};

void data_associate(ceres::Problem& problem, ceres::CostFunction* point_cost_function,
                    BALProblem& bal_problem);

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    if(argc!=2){
        std::cerr << "usage: simple_bundle_adjuster <bal_problem>\n" ;
        return 1;
    }
    BALProblem bal_problem;

    if(!bal_problem.LoadFile(argv[1])){
        std::cerr << "ERROR: UNABLE TO LOAD FILE" << argv[1] << std::endl;
        return 1;
    }

    ceres::Problem problem;
    ceres::CostFunction* point_cost_function;
    data_associate(problem, point_cost_function, bal_problem);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    return 0;
}

void data_associate(ceres::Problem& problem, ceres::CostFunction* point_cost_function,
                    BALProblem& bal_problem){
    const double* observations = bal_problem.observations();
    for (int i = 0; i < bal_problem.num_observations(); ++i) {
        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.
        point_cost_function = snavely_reprojection_error::Create(observations[2 * i + 0],
                                                   observations[2 * i + 1]);
        problem.AddResidualBlock(point_cost_function,
                                 NULL /* squared loss */,
                                 bal_problem.mutable_camera_for_observation(i),
                                 bal_problem.mutable_points_for_observation(i));
    }
}