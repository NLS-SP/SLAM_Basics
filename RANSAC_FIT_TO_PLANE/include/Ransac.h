//
// Created by Robotics_qi on 2020/9/23.
//

#ifndef RANSAC_LIB_H
#define RANSAC_LIB_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace common{
    namespace math{
        class RansacOptions{
        public:
            RansacOptions():
            min_num_iterations_(100u),
            max_num_iterations_(10000u),
            success_probability_(0.9999),
            squared_inlier_threshold_(1.0),
            random_seed_(0u){}

            uint32_t min_num_iterations_;
            uint32_t max_num_iterations_;
            double success_probability_;
            double squared_inlier_threshold_;
            unsigned int random_seed_;
        };

        class LORansacOptions : public RansacOptions{
        public:
            int num_to_steps_;
            double threshold_multiplier_;
            int num_lsq_iterations_;

            int min_sample_multiplicator_;

            int non_min_sample_multiplier_;

            uint32_t lo_starting_iterations_;

            bool final_least_squares;
        };

        struct RansacStatistics{
            uint32_t num_iteratons;
            int best_num_inliers;
            double best_model_score;
            double inlier_ratio;
            std::vector<int> inliner_indices;
            int number_lo_iterations;
        };

        class RansacBase{
        protected:
            void ResetStatistics(RansacStatistics* statistics) const{
                RansacStatistics& stats = *statistics;
                stats.best_num_inliers = 0;
                stats.best_model_score = std::numeric_limits<double>::max();
                stats.num_iteratons = 0u;
                stats.inlier_ratio = 0.0;
                stats.inliner_indices.clear();
                stats.number_lo_iterations = 0;
            }
        };

        template<class Model, class ModelVector ,class Solver, class Sampler>
        class LocallyOptimizedMSAC : public RansacBase {
        public:
            int EstimateModel(const LORansacOptions &options, const Solver &solver, Model *best_model,
                              RansacStatistics *statics) const {

            }

            void GetBestEstimatedModelID(const Solver &solver, const ModelVector &model, const int num_models,
                                         const double squared_inliner_threshold,
                                         double *best_score, int *best_model_id) const;

            void ScoreModel(const Solver &solver, const Model &model, const double squared_inlier_threshold,
                            double *score) const;

            inline double ComputeScore(const double squared_error, const double squared_error_threshold) const;

            int GetInlier(const Solver& solver, const Model& model,
                          const double squared_inlier_threshold,
                          std::vector<int>* inliers) const;

            void LocalOptimization(const LORansacOptions& options, const Solver& solver,
                                   std::mt19937* rng, Model* best_minimal_model,
                                   double* score_best_minimal_model) const;

            void LeastSquaresFit(const LORansacOptions& options, const double thresh,
                                 const Solver& solver, std::mt19937* rng, Model* model) const;
        };
    }
}

#endif //RANSAC_LIB_H
