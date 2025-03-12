#pragma once

#include <vector>
#include "feature_point.hpp"

namespace matches_msg_types {

/**
 * @brief Represents a sequence (tracklet) of feature points.
 *
 * Contains a collection of feature points along with metadata such as a unique identifier,
 * age, an outlier flag, and a label.
 */
struct Tracklet {
    std::vector<FeaturePoint> feature_points; ///< Container for feature points in the tracklet

    unsigned long id{0};   ///< Unique identifier for the tracklet
    unsigned long age{0};  ///< Age of the tracklet (e.g., number of frames or time steps)
    bool is_outlier{false};///< Flag indicating whether the tracklet is considered an outlier
    int label{-2};         ///< Label associated with the tracklet (default -2)
};

} // namespace matches_msg_types
