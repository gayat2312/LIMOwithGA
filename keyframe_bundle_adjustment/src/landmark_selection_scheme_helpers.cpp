#pragma once

#include "internal/landmark_selection_scheme_helpers.hpp"

#include <chrono>
#include <exception>
#include <fstream>
#include <random>
#include <sstream>
#include <algorithm>
#include <map>
#include <vector>
#include <iostream>
#include <iterator>

namespace keyframe_bundle_adjustment {
namespace landmark_helpers {

// Choose near landmarks based on highest flow values.
std::vector<LandmarkId> chooseNearLmIds(size_t max_num_lms,
                                        const std::vector<LandmarkId>& near_ids,
                                        const std::map<LandmarkId, double>& map_flow) {
    // Filter out landmark IDs not present in the flow map.
    std::vector<LandmarkId> ids;
    for (const auto& id : near_ids) {
        if (map_flow.find(id) != map_flow.end()) {
            ids.push_back(id);
        }
    }

    size_t num_to_select = std::min(max_num_lms, ids.size());
    std::vector<LandmarkId> out(num_to_select);

    // Partial sort copy using descending flow.
    auto num_selected = std::partial_sort_copy(
        ids.cbegin(), ids.cend(), out.begin(), out.end(),
        [&map_flow](LandmarkId a, LandmarkId b) {
            return map_flow.at(a) > map_flow.at(b);
        });
    if (num_selected != out.end()) {
        throw std::runtime_error("In LandmarkSelectionSchemeHelpers: Not all chosen IDs of near field have been copied!");
    }
    return out;
}

// Choose middle landmarks by shuffling and selecting the first max_num elements.
std::vector<LandmarkId> chooseMiddleLmIds(size_t max_num, const std::vector<LandmarkId>& middle_ids) {
    size_t num = std::min(max_num, middle_ids.size());
    std::vector<LandmarkId> shuffled = middle_ids; // copy
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(shuffled.begin(), shuffled.end(), gen);

    std::vector<LandmarkId> out;
    out.reserve(num);
    std::copy(shuffled.cbegin(), shuffled.cbegin() + num, std::back_inserter(out));
    return out;
}

// Choose far landmarks by counting measurements in keyframes and selecting those with the highest counts.
std::vector<LandmarkId> chooseFarLmIds(size_t max_num,
                                       const std::vector<LandmarkId>& ids_far,
                                       const std::map<KeyframeId, Keyframe::ConstPtr>& keyframes) {
    std::map<LandmarkId, unsigned int> count_map;
    for (const auto& id : ids_far) {
        unsigned int count = 0;
        for (const auto& kf : keyframes) {
            if (kf.second->hasMeasurement(id)) {
                ++count;
            }
        }
        count_map[id] = count;
    }

    size_t num = std::min(max_num, ids_far.size());
    std::vector<LandmarkId> chosen_ids_far(num);
    std::partial_sort_copy(ids_far.cbegin(), ids_far.cend(),
                           chosen_ids_far.begin(), chosen_ids_far.end(),
                           [&count_map](LandmarkId a, LandmarkId b) {
                               return count_map.at(a) > count_map.at(b);
                           });
    return chosen_ids_far;
}

// Calculate flow for each valid landmark from a sorted vector of keyframes.
std::map<LandmarkId, double> calcFlow(const std::vector<LandmarkId>& valid_lm_ids,
                                      const std::vector<Keyframe::ConstPtr>& sorted_kf_ptrs,
                                      bool use_mean) {
    std::map<LandmarkId, double> out;
    for (const auto& lm_id : valid_lm_ids) {
        std::map<CameraId, Measurement> last_meas;
        std::map<CameraId, double> flow_sum;
        std::map<CameraId, int> occurrence;
        for (const auto& kf : sorted_kf_ptrs) {
            auto cur_meas = kf->getMeasurements(lm_id);
            for (const auto& cam_meas : cur_meas) {
                if (last_meas.find(cam_meas.first) != last_meas.end()) {
                    double flow = (last_meas.at(cam_meas.first).toEigen2d() - cam_meas.second.toEigen2d()).norm();
                    flow_sum[cam_meas.first] += flow;
                    occurrence[cam_meas.first] += 1;
                }
                last_meas[cam_meas.first] = cam_meas.second;
            }
        }
        if (use_mean) {
            for (auto& kv : flow_sum) {
                if (occurrence[kv.first] > 0) {
                    kv.second /= occurrence[kv.first];
                }
            }
        }
        double max_flow = 0.0;
        for (const auto& kv : flow_sum) {
            max_flow = std::max(max_flow, kv.second);
        }
        out[lm_id] = max_flow;
    }
    return out;
}

// Calculate the mean flow based on the oldest and newest measurements.
std::map<LandmarkId, double> calcMeanFlow2(const std::vector<LandmarkId>& valid_lm_ids,
                                           const std::vector<Keyframe::ConstPtr>& sorted_kf_ptrs) {
    std::map<LandmarkId, double> out;
    std::cout << "Oldest ts = " << sorted_kf_ptrs.front()->timestamp_ << std::endl;
    std::cout << "Newest ts = " << sorted_kf_ptrs.back()->timestamp_ << std::endl;

    for (const auto& lm_id : valid_lm_ids) {
        std::map<CameraId, Measurement> oldest_meas;
        TimestampNSec oldest_ts = 0;
        getMeasurementFromKf(sorted_kf_ptrs.cbegin(), sorted_kf_ptrs.cend(), lm_id, oldest_meas, oldest_ts);

        std::map<CameraId, Measurement> newest_meas;
        TimestampNSec newest_ts = 0;
        getMeasurementFromKf(sorted_kf_ptrs.crbegin(), sorted_kf_ptrs.crend(), lm_id, newest_meas, newest_ts);

        if (newest_meas.empty() || oldest_meas.empty())
            continue;

        double dt_sec = convert(newest_ts - oldest_ts);
        if (dt_sec <= 0.0) {
            std::cout << "dt = " << dt_sec << std::endl;
            std::cout << "ts = " << oldest_ts << std::endl;
            std::cout << "---------1-----------" << std::endl;
            continue;
        }

        std::vector<double> flows;
        flows.reserve(oldest_meas.size() * newest_meas.size());
        for (const auto& last_pair : newest_meas) {
            for (const auto& first_pair : oldest_meas) {
                flows.push_back((last_pair.second.toEigen2d() - first_pair.second.toEigen2d()).norm() / dt_sec);
            }
        }
        double max_flow = *std::max_element(flows.begin(), flows.end());
        out[lm_id] = max_flow;
    }
    return out;
}

// Overload: calculate flow from keyframes provided as a map.
std::map<LandmarkId, double> calcFlow(const std::vector<LandmarkId>& valid_lm_ids,
                                      const std::map<KeyframeId, Keyframe::ConstPtr>& keyframes,
                                      bool use_mean) {
    std::vector<Keyframe::ConstPtr> sorted_kf_ptrs;
    sorted_kf_ptrs.reserve(keyframes.size());
    for (const auto& kv : keyframes) {
        sorted_kf_ptrs.push_back(kv.second);
    }
    std::sort(sorted_kf_ptrs.begin(), sorted_kf_ptrs.end(), [](const auto& a, const auto& b) {
        return a->timestamp_ < b->timestamp_;
    });
    return calcFlow(valid_lm_ids, sorted_kf_ptrs, use_mean);
}

} // namespace landmark_helpers

namespace keyframe_helpers {

// Returns a sorted vector of active keyframes (newest first).
std::vector<Keyframe::ConstPtr> getSortedKeyframes(const std::map<KeyframeId, Keyframe::ConstPtr>& keyframes) {
    std::vector<Keyframe::ConstPtr> sorted;
    sorted.reserve(keyframes.size());
    for (const auto& kv : keyframes) {
        if (kv.second->is_active_) {
            sorted.push_back(kv.second);
        }
    }
    std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
        return a->timestamp_ > b->timestamp_;
    });
    return sorted;
}

} // namespace keyframe_helpers
} // namespace keyframe_bundle_adjustment
