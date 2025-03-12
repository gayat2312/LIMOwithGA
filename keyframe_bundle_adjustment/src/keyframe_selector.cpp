#include "keyframe_selector.hpp"
#include <chrono>
#include <algorithm>
#include <map>
#include <cassert>

namespace keyframe_bundle_adjustment {

void KeyframeSelector::addScheme(KeyframeSelectionSchemeBase::ConstPtr scheme) {
    selection_schemes_.push_back(static_cast<KeyframeSchemeBase::ConstPtr>(scheme));
}

void KeyframeSelector::addScheme(KeyframeRejectionSchemeBase::ConstPtr scheme) {
    rejection_schemes_.push_back(static_cast<KeyframeSchemeBase::ConstPtr>(scheme));
}

void KeyframeSelector::addScheme(KeyframeSparsificationSchemeBase::ConstPtr scheme) {
    sparsification_schemes_.push_back(static_cast<KeyframeSchemeBase::ConstPtr>(scheme));
}

namespace {

// Applies rejection schemes to filter out keyframes that are not usable.
std::map<KeyframeId, Keyframe::Ptr> applyRejectionScheme(
    const KeyframeSelector::Keyframes& frames,
    const std::map<KeyframeId, Keyframe::Ptr>& buffer_selected_frames,
    const std::vector<KeyframeSchemeBase::ConstPtr>& schemes)
{
    std::map<KeyframeId, Keyframe::Ptr> selected;
    KeyframeId id = 0;
    for (const auto& frame : frames) {
        bool is_rejected = false;
        for (const auto& scheme : schemes) {
            if (!scheme->isUsable(frame, buffer_selected_frames) ||
                !scheme->isUsable(frame, selected))
            {
                is_rejected = true;
                break;
            }
        }
        if (!is_rejected) {
            selected[id++] = frame;
        }
    }
    return selected;
}

// Applies selection schemes to choose keyframes.
std::map<KeyframeId, Keyframe::Ptr> applySelectionScheme(
    const KeyframeSelector::Keyframes& frames,
    const std::map<KeyframeId, Keyframe::Ptr>& buffer_selected_frames,
    const std::vector<KeyframeSchemeBase::ConstPtr>& schemes)
{
    std::map<KeyframeId, Keyframe::Ptr> selected;
    KeyframeId id = 0;
    for (const auto& frame : frames) {
        for (const auto& scheme : schemes) {
            if (scheme->isUsable(frame, buffer_selected_frames) ||
                scheme->isUsable(frame, selected))
            {
                selected[id++] = frame;
                break;
            }
        }
    }
    return selected;
}

// Erases keyframes that were not selected by the rejection schemes.
void eraseRejected(std::map<KeyframeId, Keyframe::Ptr>& current,
                   const std::map<KeyframeId, Keyframe::Ptr>& non_rejected)
{
    if (non_rejected.empty()) {
        current.clear();
        return;
    }
    for (auto it = current.begin(); it != current.end(); ) {
        if (non_rejected.find(it->first) == non_rejected.cend()) {
            it = current.erase(it);
        } else {
            ++it;
        }
    }
}

} // anonymous namespace

KeyframeSelector::Keyframes KeyframeSelector::select(
    const Keyframes& frames,
    const std::map<KeyframeId, Keyframe::Ptr>& buffer_selected_frames)
{
    using KeyframeMap = std::map<KeyframeId, Keyframe::Ptr>;

    // Apply rejection schemes.
    KeyframeMap non_rejected = applyRejectionScheme(frames, buffer_selected_frames, rejection_schemes_);
    // Apply selection schemes.
    KeyframeMap selected = applySelectionScheme(frames, buffer_selected_frames, selection_schemes_);
    // Remove any keyframes that were rejected.
    eraseRejected(selected, non_rejected);

    // Merge the selected keyframes into the output set.
    Keyframes out;
    for (const auto& [id, frame] : selected) {
        out.insert(frame);
    }

    // For sparsification, apply the sparsification schemes and merge the result.
    for (const auto& scheme : sparsification_schemes_) {
        KeyframeMap sparsified = applyRejectionScheme(frames, buffer_selected_frames, sparsification_schemes_);
        eraseRejected(sparsified, non_rejected);
        for (const auto& [id, frame] : sparsified) {
            out.insert(frame);
        }
    }

    return out;
}

} // namespace keyframe_bundle_adjustment
