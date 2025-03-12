// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/keyframe_rejection_scheme_flow.hpp"
#include <algorithm>
#include <iostream>
#include <memory>

namespace keyframe_bundle_adjustment {

KeyframeRejectionSchemeFlow::KeyframeRejectionSchemeFlow(double min_median_flow)
    : min_median_flow_squared_(min_median_flow * min_median_flow)
{
}

bool KeyframeRejectionSchemeFlow::isUsable(const Keyframe::Ptr& new_frame,
                                           const std::map<KeyframeId, Keyframe::Ptr>& last_frames) const {
    // If no previous keyframes, allow new frame.
    if (last_frames.empty()) {
        return true;
    }
    // Reject if new frame has no measurements.
    if (new_frame->measurements_.empty()) {
        return false;
    }
    
    // Find the keyframe with the latest timestamp.
    auto it = std::max_element(last_frames.cbegin(), last_frames.cend(),
                               [](const auto& a, const auto& b) {
                                   return a.second->timestamp_ < b.second->timestamp_;
                               });
    const Keyframe::Ptr& last_keyframe = it->second;

    // Compute squared flow for each measurement between new_frame and last_keyframe.
    std::vector<double> flow_squared;
    flow_squared.reserve(new_frame->measurements_.size());

    for (const auto& m : new_frame->measurements_) {
        const auto& lm_id = m.first;
        const auto& cam_measurements = m.second;
        for (const auto& cam_meas : cam_measurements) {
            if (last_keyframe->hasMeasurement(lm_id, cam_meas.first)) {
                const Measurement& last_meas = last_keyframe->getMeasurement(lm_id, cam_meas.first);
                Eigen::Vector2d diff = cam_meas.second.toEigen2d() - last_meas.toEigen2d();
                flow_squared.push_back(diff.squaredNorm());
            }
        }
    }

    // Compute mean flow (squared). Note: variable name "median_flow" is kept for legacy.
    double sum = 0.0;
    for (const auto& val : flow_squared) {
        sum += std::sqrt(val);
    }
    sum /= static_cast<double>(flow_squared.size());
    double median_flow = sum * sum;

    if (median_flow < min_median_flow_squared_) {
        std::cout << "---- Not enough flow, reject frame." << std::endl;
    }
    return median_flow > min_median_flow_squared_;
}

KeyframeRejectionSchemeBase::ConstPtr KeyframeRejectionSchemeFlow::createConst(double min_median_flow) {
    return std::make_shared<KeyframeRejectionSchemeFlow>(min_median_flow);
}

KeyframeRejectionSchemeBase::Ptr KeyframeRejectionSchemeFlow::create(double min_median_flow) {
    return std::make_shared<KeyframeRejectionSchemeFlow>(min_median_flow);
}

} // namespace keyframe_bundle_adjustment
