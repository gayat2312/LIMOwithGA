#include "internal/keyframe_selection_scheme_pose.hpp"
#include <algorithm>
#include <iostream>
#include <memory>

namespace keyframe_bundle_adjustment {

KeyframeSelectionSchemePose::KeyframeSelectionSchemePose(double critical_quaternion_difference)
    : critical_quaternion_diff_(critical_quaternion_difference)
{
}

bool KeyframeSelectionSchemePose::isUsable(const Keyframe::Ptr& new_frame,
                                           const std::map<KeyframeId, Keyframe::Ptr>& last_frames) const {
    // If no keyframes exist, do not select the new frame.
    if (last_frames.empty()) {
        return false;
    }

    // Find the keyframe with the latest timestamp.
    auto it = std::max_element(last_frames.cbegin(), last_frames.cend(),
                               [](const auto& a, const auto& b) {
                                   return a.second->timestamp_ < b.second->timestamp_;
                               });
    const auto& last_keyframe = it->second;

    // Calculate the quaternion difference between the new frame and the last keyframe.
    double quat_diff = calcQuaternionDiff(new_frame->pose_, last_keyframe->pose_);
    std::cout << "quaternion diff = " << quat_diff << std::endl;

    // Return true if the difference exceeds the critical threshold.
    return quat_diff > critical_quaternion_diff_;
}

KeyframeSelectionSchemeBase::ConstPtr KeyframeSelectionSchemePose::createConst(double critical_quaternion_difference) {
    return std::make_shared<KeyframeSelectionSchemePose>(critical_quaternion_difference);
}

KeyframeSelectionSchemeBase::Ptr KeyframeSelectionSchemePose::create(double critical_quaternion_difference) {
    return std::make_shared<KeyframeSelectionSchemePose>(critical_quaternion_difference);
}

} // namespace keyframe_bundle_adjustment
