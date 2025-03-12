// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/landmark_selection_scheme_add_depth.hpp"
#include <algorithm>
#include "internal/landmark_selection_scheme_helpers.hpp"

namespace keyframe_bundle_adjustment {

std::set<LandmarkId> LandmarkSelectionSchemeAddDepth::getSelection(
    const LandmarkSchemeBase::LandmarkMap& landmarks,
    const LandmarkSchemeBase::KeyframeMap& keyframes) const
{
    std::set<LandmarkId> out;

    // Get keyframes sorted in descending order (newest first) then reverse for oldest-first.
    auto kf_ptrs_sorted_reverse = keyframe_helpers::getSortedKeyframes(keyframes);
    std::reverse(kf_ptrs_sorted_reverse.begin(), kf_ptrs_sorted_reverse.end());

    // Process each frame's parameters.
    for (const auto& param_tuple : params_.params_per_keyframe) {
        FrameIndex ind;
        NumberLandmarks num_lms_to_add;
        Comparator comparator;
        Sorter sorter;
        std::tie(ind, num_lms_to_add, comparator, sorter) = param_tuple;

        if (ind >= static_cast<int>(kf_ptrs_sorted_reverse.size()))
            continue;

        // Collect landmark IDs from keyframe 'ind' that satisfy the comparator.
        std::vector<LandmarkId> ids;
        for (const auto& lm_pair : kf_ptrs_sorted_reverse[ind]->measurements_) {
            if (landmarks.find(lm_pair.first) != landmarks.cend() &&
                comparator(landmarks.at(lm_pair.first)))
            {
                ids.push_back(lm_pair.first);
            }
        }

        // For each candidate landmark, compute a cost based on the measurement errors.
        std::vector<std::pair<LandmarkId, double>> ids_sort_val;
        ids_sort_val.reserve(ids.size());
        for (const auto& id : ids) {
            // Transform landmark to keyframe coordinate system.
            Eigen::Vector3d local_lm = kf_ptrs_sorted_reverse[ind]->getEigenPose() *
                                         LmMap(landmarks.at(id)->pos.data());
            std::vector<double> cost;
            for (const auto& cam_meas : kf_ptrs_sorted_reverse[ind]->measurements_.at(id)) {
                cost.push_back(sorter(cam_meas.second, local_lm));
            }
            double max_cost = *std::max_element(cost.cbegin(), cost.cend());
            ids_sort_val.push_back({id, max_cost});
        }

        // Determine how many landmarks to select (lowest cost).
        int incr = std::min(num_lms_to_add, static_cast<int>(ids_sort_val.size()));
        std::partial_sort(ids_sort_val.begin(), std::next(ids_sort_val.begin(), incr), ids_sort_val.end(),
                          [](const auto& a, const auto& b) { return a.second < b.second; });

        // Insert selected landmark IDs into the output set.
        std::transform(ids_sort_val.begin(), std::next(ids_sort_val.begin(), incr),
                       std::inserter(out, out.begin()),
                       [](const auto& pair) { return pair.first; });
    }

    return out;
}

LandmarkSelectionSchemeBase::ConstPtr LandmarkSelectionSchemeAddDepth::createConst(
    LandmarkSelectionSchemeAddDepth::Parameters p) {
    return LandmarkSelectionSchemeBase::ConstPtr(new LandmarkSelectionSchemeAddDepth(p));
}

LandmarkSelectionSchemeBase::Ptr LandmarkSelectionSchemeAddDepth::create(
    LandmarkSelectionSchemeAddDepth::Parameters p) {
    return LandmarkSelectionSchemeBase::Ptr(new LandmarkSelectionSchemeAddDepth(p));
}

} // namespace keyframe_bundle_adjustment
