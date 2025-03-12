#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <ceres/ceres.h>
#include "internal/definitions.hpp"

namespace robust_optimization {

// Aliases for residual identifiers and sizes.
using ResidualId         = ceres::ResidualBlockId;
using ResidualBlockSize  = int;
using ResidualIds        = std::vector<std::pair<ResidualId, ResidualBlockSize>>;
using ResidualGroupId    = unsigned long; ///< Identifier type for residual groups

/// @brief Map that associates each residual id with a pair of group id and residual block size.
///        This is useful in problems where a single outlier residual should lead to the rejection of
///        all measurements corresponding to that group (e.g. each landmark in bundle adjustment).
using ResidualIdMap = std::map<ResidualId, std::pair<ResidualGroupId, ResidualBlockSize>>;

/**
 * @brief Specifications for the trimmer.
 *
 * @param type The type of the trimming algorithm.
 * @param parameter Parameter for the algorithm (currently only one is supported).
 */
struct TrimmerSpecification {
    TrimmerSpecification() = default;
    TrimmerSpecification(TrimmerType type, double parameter)
      : type(type), parameter(parameter) {}

    TrimmerType type;
    double      parameter;
};

/**
 * @brief Summary struct that stores several Ceres summaries from solveTrimmed.
 *
 * This struct mimics the interface of Ceres summaries.
 */
struct Summary {
    Summary(const std::vector<ceres::Solver::Summary>& summaries) {
        Set(summaries);
    }

    std::vector<ceres::Solver::Summary> all_summaries;
    std::string merged_full_summaries;
    double      initial_cost{0.0};
    double      final_cost{0.0};
    double      time_sec{0.0};

    /**
     * @brief Returns a full report merging all summaries.
     *
     * @return std::string The merged report including solver duration.
     */
    std::string FullReport() const {
        std::stringstream ss;
        ss << merged_full_summaries << std::endl;
        ss << "Duration solveTrimmed = " << time_sec << " sec" << std::endl;
        return ss.str();
    }

    /**
     * @brief Merges summaries from the solver.
     *
     * @param summaries A vector of Ceres solver summaries.
     */
    void Set(const std::vector<ceres::Solver::Summary>& summaries) {
        std::stringstream ss;
        ss << "Merged summaries:\n";
        for (size_t i = 0; i < summaries.size(); ++i) {
            ss << "--------------------------------------------------\nIteration No. " << i << "\n";
            ss << summaries[i].FullReport();
        }
        all_summaries = summaries;
        merged_full_summaries = ss.str();
        if (!summaries.empty()) {
            initial_cost = summaries.front().initial_cost;
            final_cost = summaries.back().final_cost;
        }
    }
};

/**
 * @brief Options for the robust optimization.
 *
 * Inherits from ceres::Solver::Options and adds additional robust optimization parameters.
 */
struct Options : public ceres::Solver::Options {
    Options() = default;

    /// Maximum solver time for the last optimization step.
    double max_solver_time_refinement_in_seconds{this->max_solver_time_in_seconds * 4.0};

    /// Relaxation factor: increases the size of the last trust-region radius.
    /// A negative value means it is reset to default every iteration.
    double trust_region_relaxation_factor{3.0};

    /// Minimum number of residual groups required.
    size_t minimum_number_residual_groups{30};

    /// Action to perform before the solver runs.
    std::function<void(void)> pre_action{[]() { return; }};
    /// Action to perform after the solver runs.
    std::function<void(void)> post_action{[]() { return; }};
};

/**
 * @brief Returns typical solver options for a Ceres problem when solver time is critical.
 *
 * @param solver_time_sec Maximum solver time in seconds.
 * @return Options Initialized solver options.
 */
inline Options getStandardSolverOptions(double solver_time_sec) {
    Options options;
    // See: http://ceres-solver.org/solving_faqs.html for solver choices.
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 100;
    options.max_solver_time_in_seconds = solver_time_sec;
    options.minimizer_progress_to_stdout = true;
    return options;
}

/**
 * @brief Returns solver options for a fixed number of iterations.
 *
 * @param num_iterations Number of iterations to run.
 * @return Options Initialized solver options.
 */
inline Options getSolverOptionsNumIter(int num_iterations) {
    Options options = getStandardSolverOptions(100.0);
    options.max_num_iterations = num_iterations;
    return options;
}

/**
 * @brief solveTrimmed: Solves a Ceres problem robustly by trimming outlier residuals.
 *
 * Trimming means that residuals outside a threshold are rejected. The threshold is determined
 * by the provided TrimmerSpecification.
 *
 * @param number_iterations Vector defining the number of iterations for the trimmer and the solver.
 * @param ids A vector of pairs, where each pair consists of a ResidualIdMap and a TrimmerSpecification.
 *            The ResidualGroupId is used for problems where it makes sense to reject all residuals
 *            corresponding to one group (e.g., each landmark in bundle adjustment).
 * @param problem The Ceres problem to be optimized.
 * @param options Solver options.
 * @return Summary Summary of the robust optimization, including merged reports and cost information.
 */
Summary solveTrimmed(const std::vector<int>& number_iterations,
                     std::vector<std::pair<ResidualIdMap, TrimmerSpecification>>& ids,
                     ceres::Problem& problem,
                     Options options = getStandardSolverOptions(0.1));

/**
 * @brief Overload of solveTrimmed for the case with a single set of residual ids.
 *
 * Each residual id is given an independent group id.
 *
 * @param number_iterations Vector defining the number of iterations for the trimmer and the solver.
 * @param ids Vector of residual ids with their corresponding block sizes.
 * @param trimmer_specs Specifications for the trimming algorithm.
 * @param problem The Ceres problem to be optimized.
 * @param options Solver options.
 * @return Summary Summary of the robust optimization.
 */
inline Summary solveTrimmed(const std::vector<int>& number_iterations,
                     const ResidualIds& ids,
                     const TrimmerSpecification& trimmer_specs,
                     ceres::Problem& problem,
                     Options options = getStandardSolverOptions(0.1)) {
    // Assign each residual its own group id if treated individually.
    ResidualIdMap group_ids;
    ResidualGroupId i = 0;
    for (const auto& el : ids) {
        group_ids[el.first] = std::make_pair(i, el.second);
        ++i;
    }

    std::vector<std::pair<ResidualIdMap, TrimmerSpecification>> ids_map{
        std::make_pair(group_ids, trimmer_specs)
    };
    return solveTrimmed(number_iterations, ids_map, problem, options);
}

} // namespace robust_optimization
