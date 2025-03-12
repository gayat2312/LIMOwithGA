#pragma once
#include <Eigen/Dense>
#include "feature_point.hpp"

namespace matches_msg_types {

struct FeaturePointDepth : public FeaturePoint {
    // Default constructor.
    FeaturePointDepth() = default;

    // Construct from an Eigen::Vector3d (explicit to avoid unintended conversions).
    explicit FeaturePointDepth(const Eigen::Vector3d& p)
        : FeaturePoint(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])) {}

    // Construct from individual float components.
    FeaturePointDepth(float u, float v, float d)
        : FeaturePoint(u, v, d) {}
};

} // namespace matches_msg_types
