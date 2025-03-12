#pragma once

namespace robust_optimization {

/**
 * @brief Enum class representing the available trimmer types.
 */
enum class TrimmerType {
    Fix,      ///< Fixed trimmer type.
    Quantile  ///< Quantile-based trimmer type.
};

} // namespace robust_optimization
