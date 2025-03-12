#pragma once

#include <robust_solving.hpp>
#include <internal/apply_trimmer.hpp>

namespace robust_optimization {
namespace {

// Calculate residuals for each residual block, grouped by landmark (or residual group).
inline std::map<ResidualGroupId, std::vector<Eigen::VectorXd>> calculateResiduals(
    const ResidualIdMap& res_ids,
    bool apply_loss,
    ceres::Problem& problem)
{
    if (res_ids.empty()) {
        return {};
    }

    // Configure evaluation options.
    ceres::Problem::EvaluateOptions eval_opt{};
    eval_opt.apply_loss_function = apply_loss;
    eval_opt.num_threads = 2;

    std::map<ResidualId, int> access_index;
    int ind = 0;
    for (const auto& res : res_ids) {
        int res_size = res.second.second;
        eval_opt.residual_blocks.push_back(res.first);
        access_index[res.first] = ind;
        ind += res_size;
    }

    double cost = 0.0;
    std::vector<double> residuals;
    problem.Evaluate(eval_opt, &cost, &residuals, nullptr, nullptr);

    // Map each residual block's evaluated vector to its landmark group.
    std::map<ResidualGroupId, std::vector<Eigen::VectorXd>> out;
    for (const auto& id_ind : access_index) {
        int res_size = res_ids.at(id_ind.first).second;
        int base_ind = id_ind.second;
        Eigen::VectorXd res_vec(res_size);
        for (int i = 0; i < res_size; ++i) {
            res_vec[i] = residuals[static_cast<size_t>(base_ind + i)];
        }
        // The first element of the pair is the landmark (or group) id.
        const auto& lm_id = res_ids.at(id_ind.first).first;
        out[lm_id].push_back(res_vec);
    }
    return out;
}

// Reduce residuals to scalar norm values.
inline std::map<ResidualGroupId, std::vector<double>> reduceResidualsNorm(
    const std::map<ResidualGroupId, std::vector<Eigen::VectorXd>>& lm_res)
{
    std::map<ResidualGroupId, std::vector<double>> out;
    for (const auto& pair : lm_res) {
        std::vector<double> norms;
        norms.reserve(pair.second.size());
        for (const auto& vec : pair.second) {
            norms.push_back(vec.norm());
        }
        out[pair.first] = norms;
    }
    return out;
}

// Get the maximum residual norm per landmark group.
inline std::map<ResidualGroupId, double> getMaximumResidual(
    const std::map<ResidualGroupId, std::vector<double>>& lm_res)
{
    std::map<ResidualGroupId, double> out;
    for (const auto& pair : lm_res) {
        auto max_it = std::max_element(pair.second.cbegin(), pair.second.cend());
        out[pair.first] = (max_it != pair.second.cend() ? *max_it : 0.0);
    }
    return out;
}

/**
 * @brief Determine which residual groups (landmarks) should be removed.
 *
 * This function evaluates the residuals, reduces them to scalar norms, and then applies the
 * specified trimmer to obtain a set of residual group ids to remove.
 *
 * @param trimmer_specs The trimmer specification (type and parameter).
 * @param minimum_number_groups Minimum number of groups required before removal.
 * @param problem The Ceres problem.
 * @param res_ids Residual id map, mapping residual block ids to landmark group and size.
 * @param residuals_to_remove Output set of landmark group ids to remove.
 */
inline void getResidualsToRemove(TrimmerSpecification trimmer_specs,
                                 size_t minimum_number_groups,
                                 ceres::Problem& problem,
                                 const ResidualIdMap& res_ids,
                                 std::set<ResidualGroupId>& residuals_to_remove)
{
    // Evaluate residuals without applying loss.
    auto residuals = calculateResiduals(res_ids, false, problem);

    if (residuals.size() < minimum_number_groups) {
        return;
    }

    auto reduced_res_vec = reduceResidualsNorm(residuals);
    auto reduced_res = getMaximumResidual(reduced_res_vec);

    // Use the trimmer to reject landmarks with high residuals.
    std::vector<ResidualGroupId> rejected = getOutliers(reduced_res, trimmer_specs.type, trimmer_specs.parameter);

    for (const auto& id : rejected) {
        residuals_to_remove.insert(id);
    }
}

// Remove parameter blocks that are no longer constrained by any residual blocks.
inline void removeUnconstraintParameters(ceres::Problem& problem) {
    std::vector<double*> param_blocks;
    problem.GetParameterBlocks(&param_blocks);
    for (auto pb : param_blocks) {
        std::vector<ceres::ResidualBlockId> residual_blocks;
        problem.GetResidualBlocksForParameterBlock(pb, &residual_blocks);
        if (residual_blocks.empty()) {
            problem.RemoveParameterBlock(pb);
        }
    }
}

} // anonymous namespace

Summary solveTrimmed(const std::vector<int>& number_iterations,
                     std::vector<std::pair<ResidualIdMap, TrimmerSpecification>>& ids_trimmer_specs,
                     ceres::Problem& problem,
                     Options options)
{
    auto start_time = std::chrono::steady_clock::now();
    std::vector<ceres::Solver::Summary> summaries;
    int number_iterations_final = options.max_num_iterations;
    double last_trust_region_radius = options.initial_trust_region_radius / options.trust_region_relaxation_factor;

    // Alternate between optimization and outlier rejection.
    for (const auto& num_outlier_iter : number_iterations) {
        options.max_num_iterations = num_outlier_iter;
        if (options.trust_region_relaxation_factor > 0.0) {
            options.initial_trust_region_radius = last_trust_region_radius * options.trust_region_relaxation_factor;
            options.max_trust_region_radius = std::pow(options.initial_trust_region_radius, 4);
        }

        ceres::Solver::Summary cur_summary;
        ceres::Solve(options, &problem, &cur_summary);
        double cost_change = cur_summary.initial_cost - cur_summary.final_cost;
        if (cost_change <= 0.0) {
            options.max_num_iterations = 3 * num_outlier_iter;
            ceres::Solve(options, &problem, &cur_summary);
            cost_change = cur_summary.initial_cost - cur_summary.final_cost;
            if (cost_change <= 0.0) {
                std::cout << "--------------------------------------------------\n"
                          << "Outlier rejection is unstable; problem did not reduce error after "
                          << 3 * num_outlier_iter << " iterations\n";
            }
        }
        summaries.push_back(cur_summary);

        // Execute any defined pre-action.
        options.pre_action();

        std::set<ResidualGroupId> residual_groups_to_remove;
        for (auto& pair : ids_trimmer_specs) {
            getResidualsToRemove(pair.second, options.minimum_number_residual_groups, problem, pair.first, residual_groups_to_remove);
        }

        // Remove residual blocks corresponding to rejected landmark groups.
        for (auto& ids_spec : ids_trimmer_specs) {
            auto& map_ids = ids_spec.first;
            for (auto it = map_ids.begin(); it != map_ids.end(); ) {
                if (residual_groups_to_remove.find(it->second.first) != residual_groups_to_remove.cend()) {
                    problem.RemoveResidualBlock(it->first);
                    it = map_ids.erase(it);
                } else {
                    ++it;
                }
            }
        }

        // Clean up any parameter blocks that are no longer used.
        removeUnconstraintParameters(problem);

        // Save trust region from the last successful iteration.
        for (auto it = cur_summary.iterations.crbegin(); it != cur_summary.iterations.crend(); ++it) {
            if (it->step_is_successful && it->step_is_valid) {
                last_trust_region_radius = it->trust_region_radius;
                std::cout << "Robust solving iteration: " << it->iteration << std::endl;
                break;
            }
        }

        // Execute any defined post-action.
        options.post_action();
    }

    // Restore original iteration settings.
    options.max_num_iterations = number_iterations_final;
    options.max_solver_time_in_seconds = options.max_solver_time_refinement_in_seconds;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    summaries.push_back(summary);

    Summary trimmer_summary(summaries);
    trimmer_summary.time_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now() - start_time)
                                   .count() * 1e-3;

    return trimmer_summary;
}

} // namespace robust_optimization
