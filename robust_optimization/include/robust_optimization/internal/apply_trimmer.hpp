#pragma once

#include <map>
#include <vector>
#include <stdexcept>
#include <utility>  // for std::forward

#include "definitions.hpp"
#include "trimmer_fix.hpp"
#include "trimmer_quantile.hpp"

namespace robust_optimization {

/**
 * @brief Factory function that constructs a trimmer and applies it to filter out outliers.
 *
 * If you implement a new trimmer, register it as a type and handle it in this function.
 *
 * @tparam Id Type used for identifying data points.
 * @tparam Args Additional arguments to be forwarded to the trimmer constructor.
 * @param data The data to be trimmed; keys are identifiers and values are associated double values.
 * @param type The type of filter (trimmer) to be applied.
 * @param filter_args Additional parameters forwarded to the trimmer constructor.
 * @return std::vector<Id> Vector containing the identifiers of outliers.
 * @throws std::runtime_error if the provided TrimmerType is not defined.
 */
template <typename Id, typename... Args>
std::vector<Id> getOutliers(const std::map<Id, double>& data, TrimmerType type, Args&&... filter_args) {
    switch (type) {
    case TrimmerType::Fix: {
        TrimmerFix t(std::forward<Args>(filter_args)...);
        return t.getOutliers(data);
    }
    case TrimmerType::Quantile: {
        TrimmerQuantile t(std::forward<Args>(filter_args)...);
        return t.getOutliers(data);
    }
    default:
        throw std::runtime_error("In getOutliers: TrimmerType not defined");
    }
}

} // namespace robust_optimization
