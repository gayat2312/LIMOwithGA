#pragma once

#include <map>
#include <vector>

namespace robust_optimization {

/**
 * @class TrimmerFix
 * @brief Simple concrete trimmer that uses a fixed residual threshold for outlier rejection.
 *
 * This trimmer filters out data points whose residuals exceed a predefined threshold.
 */
class TrimmerFix {
public:
    /**
     * @brief Constructor with a fixed outlier threshold.
     * @param outlier_threshold Residual threshold beyond which data points are considered outliers.
     */
    explicit TrimmerFix(double outlier_threshold) : outlier_thres_(outlier_threshold) {}

    // Default move and copy operations
    TrimmerFix(TrimmerFix&& other) noexcept = default;
    TrimmerFix& operator=(TrimmerFix&& other) noexcept = default;
    TrimmerFix(const TrimmerFix& other) = default;
    TrimmerFix& operator=(const TrimmerFix& other) = default;

    /**
     * @brief Filters out outliers based on the fixed threshold.
     * 
     * Iterates over the provided residuals map and collects the identifiers for which
     * the corresponding residual exceeds the threshold.
     *
     * @tparam Id Type of the identifier.
     * @param residuals_input A map of identifiers to residual values.
     * @return std::vector<Id> Vector containing identifiers of the outliers.
     */
    template <typename Id>
    std::vector<Id> getOutliers(const std::map<Id, double>& residuals_input) const {
        std::vector<Id> outliers;
        for (const auto& el : residuals_input) {
            if (el.second > outlier_thres_) {
                outliers.push_back(el.first);
            }
        }
        return outliers;
    }

private:
    double outlier_thres_; ///< Fixed threshold: residuals greater than this value are rejected.
};

} // namespace robust_optimization
