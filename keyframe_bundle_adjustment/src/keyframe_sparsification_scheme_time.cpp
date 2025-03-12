// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/keyframe_sparsification_scheme_time.hpp"
#include <algorithm>
#include <memory>

namespace keyframe_bundle_adjustment {

bool KeyframeSparsificationSchemeTime::isUsable(const Keyframe::Ptr& new_frame,
                                                const std::map<KeyframeId, Keyframe::Ptr>& last_frames) const {
    if (last_frames.empty()) {
        return true;
    }

    // Find the keyframe with the maximum timestamp.
    auto it = std::max_element(last_frames.cbegin(), last_frames.cend(),
                               [](const auto& a, const auto& b) {
                                   return a.second->timestamp_ < b.second->timestamp_;
                               });
    const TimestampNSec max_ts = it->second->timestamp_;
    return (new_frame->timestamp_ - max_ts) > time_difference_nano_sec_;
}

KeyframeSparsificationSchemeBase::ConstPtr KeyframeSparsificationSchemeTime::createConst(double time_difference_nano_sec_) {
    return std::make_shared<KeyframeSparsificationSchemeTime>(time_difference_nano_sec_);
}

KeyframeSparsificationSchemeBase::Ptr KeyframeSparsificationSchemeTime::create(double time_difference_nano_sec_) {
    return std::make_shared<KeyframeSparsificationSchemeTime>(time_difference_nano_sec_);
}

} // namespace keyframe_bundle_adjustment
