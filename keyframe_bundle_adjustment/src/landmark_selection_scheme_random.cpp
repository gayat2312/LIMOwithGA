#include "internal/landmark_selection_scheme_random.hpp"
#include <algorithm>
#include <random>

namespace keyframe_bundle_adjustment {

std::set<LandmarkId> LandmarkSparsificationSchemeRandom::getSelection(const LandmarkMap& landmarks,
                                                                      const KeyframeMap& /*keyframes*/) const {
    // Copy landmark IDs from the landmarks map.
    std::vector<LandmarkId> lm_ids;
    lm_ids.reserve(landmarks.size());
    std::transform(landmarks.cbegin(), landmarks.cend(), std::back_inserter(lm_ids),
                   [](const auto& a) { return a.first; });

    // Shuffle the landmark IDs using a modern random engine.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(lm_ids.begin(), lm_ids.end(), gen);

    // Select the first num_landmarks_ IDs.
    std::set<LandmarkId> selected_landmark_ids;
    auto end_it = (lm_ids.size() > num_landmarks_) ? lm_ids.begin() + num_landmarks_ : lm_ids.end();
    for (auto it = lm_ids.cbegin(); it != end_it; ++it) {
        selected_landmark_ids.insert(*it);
    }
    return selected_landmark_ids;
}

LandmarkSparsificationSchemeBase::ConstPtr LandmarkSparsificationSchemeRandom::createConst(size_t num_landmarks) {
    return std::make_shared<LandmarkSparsificationSchemeRandom>(num_landmarks);
}

LandmarkSparsificationSchemeBase::Ptr LandmarkSparsificationSchemeRandom::create(size_t num_landmarks) {
    return std::make_shared<LandmarkSparsificationSchemeRandom>(num_landmarks);
}

} // namespace keyframe_bundle_adjustment
