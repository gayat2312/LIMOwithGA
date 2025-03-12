#pragma once

#include "internal/landmark_selection_scheme_voxel.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <chrono>
#include "internal/landmark_selection_scheme_helpers.hpp"

// Type definitions.
using Point = pcl::PointXYZL;
using Cloud = pcl::PointCloud<Point>;

// Register the point type for Boost.Geometry.
BOOST_GEOMETRY_REGISTER_POINT_3D_CONST(Point, float, boost::geometry::cs::cartesian, x, y, z)
using BoostPoint = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;
using PosePath = boost::geometry::model::linestring<BoostPoint>;

namespace keyframe_bundle_adjustment {

std::set<LandmarkId> LandmarkSparsificationSchemeVoxel::getSelection(const LandmarkMap& landmarks,
                                                                     const KeyframeMap& keyframes) const {
    auto categorized = getCategorizedSelection(landmarks, keyframes);
    std::set<LandmarkId> selected;
    for (const auto& entry : categorized) {
        selected.insert(entry.first);
    }
    return selected;
}

namespace {

inline void filterXYZ(const Cloud::Ptr& cloudInput,
                      const std::array<double, 3>& bounds,
                      Cloud::Ptr& cloudFiltered,
                      std::set<int>& removed_labels) {
    // Create shared pointers for index storage.
    auto indices_x = boost::make_shared<std::vector<int>>();
    auto indices_xy = boost::make_shared<std::vector<int>>();

    pcl::PassThrough<Point> pass(true);
    // Filter along X.
    pass.setInputCloud(cloudInput);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-bounds[0] / 2.0, bounds[0] / 2.0);
    pass.filter(*indices_x);
    for (const auto& idx : *pass.getRemovedIndices()) {
        removed_labels.insert(cloudInput->points[idx].label);
    }
    // Filter along Y.
    pass.setIndices(indices_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-bounds[1] / 2.0, bounds[1] / 2.0);
    pass.filter(*indices_xy);
    for (const auto& idx : *pass.getRemovedIndices()) {
        removed_labels.insert(cloudInput->points[idx].label);
    }
    // Filter along Z.
    pass.setIndices(indices_xy);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-bounds[2] / 2.0, bounds[2] / 2.0);
    pass.filter(*cloudFiltered);
    for (const auto& idx : *pass.getRemovedIndices()) {
        removed_labels.insert(cloudInput->points[idx].label);
    }
}

inline void filterPipe(const Cloud::Ptr& cloudInput,
                       const Eigen::Isometry3d& ref_pose,
                       const LandmarkSchemeBase::KeyframeMap& keyframes,
                       double dist_thres,
                       Cloud::Ptr& cloudFiltered,
                       std::set<int>& removed_labels) {
    // Build a path (PosePath) from keyframes, transformed by ref_pose.
    PosePath path;
    for (const auto& kf : keyframes) {
        Eigen::Vector3d pos = ref_pose * kf.second->getEigenPose().inverse().translation();
        path.push_back(BoostPoint(pos.x(), pos.y(), pos.z()));
    }
    // For each point, add it to the filtered cloud if it lies within the threshold; otherwise mark it.
    for (const auto& p : cloudInput->points) {
        double distance = boost::geometry::distance(p, path);
        if (distance < dist_thres) {
            cloudFiltered->points.push_back(p);
        } else {
            removed_labels.insert(p.label);
        }
    }
}

} // anonymous namespace

std::map<LandmarkId, LandmarkCategorizatonInterface::Category> 
LandmarkSparsificationSchemeVoxel::getCategorizedSelection(const LandmarkMap& lms,
                                                           const KeyframeMap& keyframes) const {
    // Find the most recent keyframe based on timestamp.
    auto recent_it = std::max_element(keyframes.cbegin(), keyframes.cend(),
                                      [](const auto& a, const auto& b) {
                                          return a.second->timestamp_ < b.second->timestamp_;
                                      });
    Eigen::Isometry3d cur_pos = recent_it->second->getEigenPose();

    // Build a lookup table from internal indices to landmark IDs.
    std::map<uint32_t, LandmarkId> lut;
    uint32_t idx = 0;

    auto start_time = std::chrono::steady_clock::now();
    Cloud::Ptr cloudInput(new Cloud);
    for (const auto& id_lm : lms) {
        Eigen::Vector3d p_eig(id_lm.second->pos[0], id_lm.second->pos[1], id_lm.second->pos[2]);
        p_eig = cur_pos * p_eig; // Transform point to current keyframe coordinate system.
        Point p;
        p.x = static_cast<float>(p_eig.x());
        p.y = static_cast<float>(p_eig.y());
        p.z = static_cast<float>(p_eig.z());
        p.label = idx; // Use idx as temporary label.
        lut[idx] = id_lm.first;
        ++idx;
        cloudInput->points.push_back(p);
    }
    std::cout << "Size before voxelization = " << lms.size() << std::endl;

    // Apply a pass-through filter along Z to ensure plausibility.
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(cloudInput);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-20.0, 100.0);
    pass.filter(*cloudInput);

    // Filter with a larger ROI for voxelization.
    Cloud::Ptr cloud_middle(new Cloud);
    std::set<int> labels_far;
    filterPipe(cloudInput, cur_pos, keyframes, params_.roi_far_xyz[0], cloud_middle, labels_far);

    // Voxelize the middle cloud.
    pcl::VoxelGrid<Point> voxel;
    voxel.setInputCloud(cloud_middle);
    voxel.setLeafSize(params_.voxel_size_xyz[0], params_.voxel_size_xyz[1], params_.voxel_size_xyz[2]);
    voxel.filter(*cloud_middle);

    // Filter with a smaller ROI to obtain the "near" cloud.
    Cloud::Ptr cloud_near(new Cloud);
    std::set<int> labels_middle;
    filterPipe(cloud_middle, cur_pos, keyframes, params_.roi_middle_xyz[0], cloud_near, labels_middle);

    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - start_time)
                           .count();
    std::cout << "Duration for PCL processing = " << duration_ms << " ms" << std::endl;

    // Convert filtered cloud indices back to landmark IDs.
    std::vector<LandmarkId> ids_near;
    ids_near.reserve(cloud_near->points.size());
    for (const auto& p : cloud_near->points) {
        ids_near.push_back(lut.at(p.label));
    }

    // Create output categories map.
    std::map<LandmarkId, LandmarkCategorizatonInterface::Category> categorized;

    // Categorize near landmarks.
    auto near_ids = landmark_helpers::chooseNearLmIds(params_.max_num_landmarks_near, ids_near,
                                                      landmark_helpers::calcFlow(ids_near, keyframes, false));
    for (const auto& id : near_ids) {
        categorized[id] = LandmarkCategorizatonInterface::Category::NearField;
    }

    // Categorize middle landmarks.
    std::vector<LandmarkId> ids_middle;
    ids_middle.reserve(labels_middle.size());
    for (const auto& label : labels_middle) {
        ids_middle.push_back(lut.at(label));
    }
    auto middle_ids = landmark_helpers::chooseMiddleLmIds(params_.max_num_landmarks_middle, ids_middle);
    for (const auto& id : middle_ids) {
        categorized[id] = LandmarkCategorizatonInterface::Category::MiddleField;
    }

    // Categorize far landmarks.
    std::vector<LandmarkId> ids_far;
    ids_far.reserve(labels_far.size());
    for (const auto& label : labels_far) {
        ids_far.push_back(lut.at(label));
    }
    auto far_ids = landmark_helpers::chooseFarLmIds(params_.max_num_landmarks_far, ids_far, keyframes);
    for (const auto& id : far_ids) {
        categorized[id] = LandmarkCategorizatonInterface::Category::FarField;
    }

    std::cout << "After voxelization: near = " 
              << std::count_if(categorized.cbegin(), categorized.cend(), [](const auto& e) {
                     return e.second == LandmarkCategorizatonInterface::Category::NearField;
                 })
              << " middle = " 
              << std::count_if(categorized.cbegin(), categorized.cend(), [](const auto& e) {
                     return e.second == LandmarkCategorizatonInterface::Category::MiddleField;
                 })
              << " far = " 
              << std::count_if(categorized.cbegin(), categorized.cend(), [](const auto& e) {
                     return e.second == LandmarkCategorizatonInterface::Category::FarField;
                 })
              << std::endl;

    return categorized;
}

} // namespace keyframe_bundle_adjustment
