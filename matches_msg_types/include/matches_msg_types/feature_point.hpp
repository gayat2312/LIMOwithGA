#pragma once
#include <Eigen/Dense>

namespace matches_msg_types {

struct FeaturePoint {
    // Default member initializers.
    float u = 0.0f;
    float v = 0.0f;
    float d = -1.0f; // Default: depth not provided.

    // Default constructor.
    FeaturePoint() = default;

    // Construct from a 2D Eigen vector; depth remains -1.
    explicit FeaturePoint(const Eigen::Vector2d& p)
        : u(static_cast<float>(p[0])), v(static_cast<float>(p[1])), d(-1.0f) {}

    // Construct from a 3D Eigen vector.
    explicit FeaturePoint(const Eigen::Vector3d& p)
        : u(static_cast<float>(p[0])), v(static_cast<float>(p[1])), d(static_cast<float>(p[2])) {}

    // Construct from two floats (u, v); depth defaults to -1.
    FeaturePoint(float u, float v)
        : u(u), v(v), d(-1.0f) {}

    // Construct from three floats.
    FeaturePoint(float u, float v, float d)
        : u(u), v(v), d(d) {}

    // Convert to a 2D Eigen vector.
    Eigen::Vector2d toEigen2d() const {
        return Eigen::Vector2d(u, v);
    }

    // Convert to a 3D Eigen vector.
    Eigen::Vector3d toEigen3d() const {
        return Eigen::Vector3d(u, v, d);
    }
};

} // namespace matches_msg_types
