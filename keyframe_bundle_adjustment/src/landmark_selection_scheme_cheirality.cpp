// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/landmark_selection_scheme_cheirality.hpp"
#include <chrono>
#include <iostream>

namespace keyframe_bundle_adjustment {

namespace {

// Checks whether a landmark fulfills the cheirality constraint for all active keyframes.
bool is_landmark_cheiral(const LandmarkSchemeBase::KeyframeMap& keyframes,
                         const std::pair<LandmarkId, Landmark::ConstPtr>& lm) {
    for (const auto& [kf_id, kf_ptr] : keyframes) {
        if (kf_ptr->is_active_) {
            // Test cheirality for every projected landmark in all cameras.
            for (const auto& cam_lm : kf_ptr->getProjectedLandmarkPosition(lm)) {
                if (cam_lm.second.z() < 0.0) {
                    return false;
                }
            }
        }
    }
    return true;
}

} // anonymous namespace

std::set<LandmarkId> LandmarkRejectionSchemeCheirality::getSelection(
    const LandmarkSchemeBase::LandmarkMap& landmarks,
    const LandmarkSchemeBase::KeyframeMap& keyframes) const {

    std::set<LandmarkId> selected;
    auto start_time = std::chrono::steady_clock::now();

    // Iterate over all landmarks and select those that fulfill the cheirality constraint.
    for (const auto& lm_el : landmarks) {
        if (is_landmark_cheiral(keyframes, lm_el)) {
            selected.insert(lm_el.first);
        }
    }

    std::cout << "Duration cheirality check = "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - start_time).count()
              << " ms" << std::endl;

    return selected;
}

LandmarkRejectionSchemeBase::ConstPtr LandmarkRejectionSchemeCheirality::createConst() {
    return std::make_shared<LandmarkRejectionSchemeCheirality>();
}

LandmarkRejectionSchemeBase::Ptr LandmarkRejectionSchemeCheirality::create() {
    return std::make_shared<LandmarkRejectionSchemeCheirality>();
}

} // namespace keyframe_bundle_adjustment
