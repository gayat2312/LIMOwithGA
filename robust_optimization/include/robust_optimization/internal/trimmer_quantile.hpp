#pragma once

#include <algorithm>
#include <map>
#include <vector>

namespace robust_optimization {

/**
 * @class TrimmerQuantile
 * @brief Trims data using quantiles.
 *
 * If quantile = 0.9, then the highest 10% of residuals are considered outliers.
 */
class TrimmerQuantile {
public:
    /**
     * @brief Constructor for the quantile trimmer.
     * @param quantile Quantile for rejection (should be between 0 and 1).
     */
    explicit TrimmerQuantile(double quantile) : quantile_(quantile) {}

    // Default move and copy operations.
    TrimmerQuantile(TrimmerQuantile&& other) noexcept = default;
    TrimmerQuantile& operator=(TrimmerQuantile&& other) noexcept = default;
    TrimmerQuantile(const TrimmerQuantile& other) = default;
    TrimmerQuantile& operator=(const TrimmerQuantile& other) = default;

    /**
     * @brief Filters the provided residuals and returns identifiers of outliers.
     *
     * Copies the input map into a vector, partitions it based on the specified quantile,
     * and then returns the identifiers corresponding to the highest residuals.
     *
     * @tparam Id Type used for identifying data points.
     * @param residuals_input Map from identifiers to residual values.
     * @return std::vector<Id> Vector of identifiers of outliers.
     */
    template <typename Id>
    std::vector<Id> getOutliers(const std::map<Id, double>& residuals_input) const {
        // Copy the map entries into a vector for sorting.
        std::vector<std::pair<Id, double>> input_vec(residuals_input.begin(), residuals_input.end());

        // Calculate the index corresponding to the quantile threshold.
        size_t index = static_cast<size_t>(input_vec.size() * quantile_);

        // Partition the vector such that elements from 'index' onward have the highest residuals.
        std::nth_element(input_vec.begin(), input_vec.begin() + index, input_vec.end(),
                         [](const auto& a, const auto& b) {
                             return a.second < b.second;
                         });

        // Collect the identifiers of the outliers.
        std::vector<Id> outliers;
        for (auto it = input_vec.cbegin() + index; it != input_vec.cend(); ++it) {
            outliers.push_back(it->first);
        }
        return outliers;
    }

private:
    double quantile_; ///< Quantile for rejection (e.g., 0.9 means 10% largest residuals are rejected).
};

} // namespace robust_optimization
