#pragma once

#include <Eigen/Eigen>
#include <keyframe_bundle_adjustment/bundle_adjuster_keyframes.hpp>

// For publishing landmarks.
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <set>
#include <tuple>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <array>

namespace keyframe_bundle_adjustment_ros_tool {
  
namespace helpers {

/**
 * @brief Converts an Eigen transform into a geometry_msgs::Pose message.
 * 
 * @tparam TransformType Must be either Eigen::Affine or Eigen::Isometry.
 * @param out Output geometry_msgs::Pose.
 * @param pose Input Eigen transform.
 */
template <int TransformType>
inline void toGeometryMsg(geometry_msgs::Pose &out,
                          const Eigen::Transform<double, 3, TransformType>& pose) {
    Eigen::Quaterniond quat(pose.rotation());
    out.position.x = pose.translation().x();
    out.position.y = pose.translation().y();
    out.position.z = pose.translation().z();
    out.orientation.x = quat.x();
    out.orientation.y = quat.y();
    out.orientation.z = quat.z();
    out.orientation.w = quat.w();
}

/**
 * @brief Converts an Eigen transform into a geometry_msgs::Transform message.
 * 
 * @tparam TransformType Must be either Eigen::Affine or Eigen::Isometry.
 * @param out Output geometry_msgs::Transform.
 * @param pose Input Eigen transform.
 */
template <int TransformType>
inline void toGeometryMsg(geometry_msgs::Transform &out,
                          const Eigen::Transform<double, 3, TransformType>& pose) {
    Eigen::Quaterniond quat(pose.rotation());
    out.translation.x = pose.translation().x();
    out.translation.y = pose.translation().y();
    out.translation.z = pose.translation().z();
    out.rotation.x = quat.x();
    out.rotation.y = quat.y();
    out.rotation.z = quat.z();
    out.rotation.w = quat.w();
}

namespace {
  using Category = keyframe_bundle_adjustment::LandmarkCategorizatonInterface::Category;
  
  /**
   * @brief Computes a color value based on landmark category.
   * 
   * Outliers (missing a category) default to a value of 80.
   * 
   * @param lm_id Landmark ID.
   * @param lm_categories Map from landmark IDs to their category.
   * @return float A value used to color-code the landmark.
   */
  inline float getColorVal(const keyframe_bundle_adjustment::LandmarkId& lm_id,
                           const std::map<keyframe_bundle_adjustment::LandmarkId, Category>& lm_categories) {
      float val = 80.f;
      auto it = lm_categories.find(lm_id);
      if (it != lm_categories.cend()) {
          switch (it->second) {
              case Category::FarField:
                  val = 250.f;
                  break;
              case Category::MiddleField:
                  val = 180.f;
                  break;
              case Category::NearField:
                  val = 120.f;
                  break;
              default:
                  break;
          }
      }
      return val;
  }
}

/**
 * @brief Publishes landmarks as a sensor_msgs::PointCloud2 message.
 * 
 * Landmarks are color-coded based on their category and measurement status.
 * 
 * @param landmarks_publisher ROS publisher for the landmark point cloud.
 * @param bundle_adjuster Pointer to the bundle adjuster holding keyframes and landmarks.
 * @param tf_parent_frame_id The global frame id for the published cloud.
 */
inline void publishLandmarks(ros::Publisher& landmarks_publisher,
                             keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster,
                             const std::string& tf_parent_frame_id) {
    using PointType = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<PointType>;
    
    // Use timestamp from the last keyframe.
    ros::Time stamp;
    stamp.fromNSec(bundle_adjuster->getKeyframe().timestamp_);
    
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = "/" + tf_parent_frame_id;
    cloud->header.stamp = stamp.toNSec();
    cloud->height = 1;
    
    // Get selected landmarks (inliers) and active landmarks.
    const auto& selected_lms = bundle_adjuster->getSelectedLandmarkConstPtrs();
    const auto& active_lms = bundle_adjuster->getActiveLandmarkConstPtrs();
    cloud->width = active_lms.size();
    
    // Retrieve categories to color-code landmarks.
    auto lm_categories = bundle_adjuster->landmark_selector_->getLandmarkCategories();
    
    // Plot inliers in green (or a mix if ground plane or depth available).
    for (const auto& lm_pair : selected_lms) {
        const auto& lm = *(lm_pair.second);
        float val = getColorVal(lm_pair.first, lm_categories);
        std::array<float, 3> color_rgb = {0.f, val, 0.f};
        if (lm.is_ground_plane) {
            color_rgb = {0.f, val, val};
        }
        if (lm.has_measured_depth) {
            color_rgb = {val, val, 0.f};
        }
        
        PointType point;
        point.x = lm.pos[0];
        point.y = lm.pos[1];
        point.z = lm.pos[2];
        point.r = static_cast<uint8_t>(color_rgb[0]);
        point.g = static_cast<uint8_t>(color_rgb[1]);
        point.b = static_cast<uint8_t>(color_rgb[2]);
        cloud->points.push_back(point);
    }
    
    // Compute difference between active and selected landmarks to determine outliers.
    std::map<keyframe_bundle_adjustment::LandmarkId, keyframe_bundle_adjustment::Landmark::ConstPtr> diff;
    std::set_difference(active_lms.cbegin(), active_lms.cend(),
                        selected_lms.cbegin(), selected_lms.cend(),
                        std::inserter(diff, diff.begin()),
                        [](const auto& a, const auto& b) { return a.first < b.first; });
    
    // Plot outliers in red.
    for (const auto& lm_pair : diff) {
        const auto& lm = *lm_pair.second;
        std::array<float, 3> color_rgb = {100.f, 0.f, 0.f};
        if (lm.has_measured_depth) {
            color_rgb = {100.f, 0.f, 100.f};
        }
        
        PointType point;
        point.x = lm.pos[0];
        point.y = lm.pos[1];
        point.z = lm.pos[2];
        point.r = static_cast<uint8_t>(color_rgb[0]);
        point.g = static_cast<uint8_t>(color_rgb[1]);
        point.b = static_cast<uint8_t>(color_rgb[2]);
        cloud->points.push_back(point);
    }
    
    landmarks_publisher.publish(cloud);
}

/**
 * @brief Publishes ground plane markers as a visualization_msgs::MarkerArray.
 * 
 * Markers are created for each keyframe with a valid local ground plane.
 * 
 * @param plane_publisher ROS publisher for the markers.
 * @param bundle_adjuster Pointer to the bundle adjuster containing keyframe data.
 * @param tf_parent_frame_id The frame id for the markers.
 */
inline void publishPlanes(ros::Publisher& plane_publisher,
                          keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster,
                          const std::string& tf_parent_frame_id) {
    visualization_msgs::MarkerArray marker_array;
    double plane_size = 6.0;
    int count = 0;
    
    for (const auto& kf_pair : bundle_adjuster->keyframes_) {
        const auto& kf = kf_pair.second;
        if (kf->local_ground_plane_.distance > -0.1) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = tf_parent_frame_id;
            ros::Time ts;
            ts.fromNSec(kf->timestamp_);
            marker.header.stamp = ts;
            marker.ns = std::to_string(count);
            marker.id = count;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = plane_size;
            marker.scale.y = plane_size;
            marker.scale.z = 0.1;
            marker.color.a = 0.3; // Set transparency.
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            
            // Determine plane orientation from the plane's direction vector.
            Eigen::Map<const Eigen::Vector3d> plane_dir(kf->local_ground_plane_.direction.data());
            Eigen::Quaterniond ori;
            ori.setFromTwoVectors(Eigen::Vector3d(0., 0., 1.), plane_dir);
            marker.pose.orientation.x = ori.x();
            marker.pose.orientation.y = ori.y();
            marker.pose.orientation.z = ori.z();
            marker.pose.orientation.w = ori.w();
            
            // Compute the position offset for the plane.
            Eigen::Vector3d pos = kf->getEigenPose().inverse().translation() - plane_dir * kf->local_ground_plane_.distance;
            marker.pose.position.x = pos.x();
            marker.pose.position.y = pos.y();
            marker.pose.position.z = pos.z();
            
            marker_array.markers.push_back(marker);
            count++;
            
            // Also add an arrow marker.
            visualization_msgs::Marker arrow_marker;
            arrow_marker.header.frame_id = tf_parent_frame_id;
            arrow_marker.header.stamp = ts;
            arrow_marker.ns = std::to_string(count);
            arrow_marker.id = count;
            arrow_marker.type = visualization_msgs::Marker::ARROW;
            arrow_marker.action = visualization_msgs::Marker::ADD;
            arrow_marker.scale.x = 3.0;
            arrow_marker.scale.y = 0.3;
            arrow_marker.scale.z = 0.3;
            arrow_marker.color.a = 0.7;
            arrow_marker.color.r = 0.5;
            arrow_marker.color.g = 0.5;
            arrow_marker.color.b = 0.0;
            
            Eigen::Quaterniond arrow_ori;
            arrow_ori.setFromTwoVectors(Eigen::Vector3d(1., 0., 0.), plane_dir);
            arrow_marker.pose.orientation.x = arrow_ori.x();
            arrow_marker.pose.orientation.y = arrow_ori.y();
            arrow_marker.pose.orientation.z = arrow_ori.z();
            arrow_marker.pose.orientation.w = arrow_ori.w();
            arrow_marker.pose.position = marker.pose.position;
            
            marker_array.markers.push_back(arrow_marker);
            count++;
        }
    }
    
    plane_publisher.publish(marker_array);
}

/**
 * @brief Publishes two nav_msgs::Path messages:
 * one representing all keyframes and one representing active keyframes.
 * 
 * The header timestamp is set to the timestamp of the most recent keyframe,
 * and the frame id is set to the provided tf_parent_frame_id.
 * 
 * @param path_publisher Publisher for the full path.
 * @param active_path_publisher Publisher for the active path.
 * @param bundle_adjuster Pointer to the bundle adjuster containing keyframe data.
 * @param tf_parent_frame_id The reference frame for the paths.
 */
inline void publishPaths(ros::Publisher& path_publisher,
                         ros::Publisher& active_path_publisher,
                         keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster,
                         const std::string& tf_parent_frame_id) {
    nav_msgs::Path full_path, active_path;
    ros::Time current_ts;
    current_ts.fromNSec(bundle_adjuster->getKeyframe().timestamp_);
    full_path.header.stamp = current_ts;
    full_path.header.frame_id = "/" + tf_parent_frame_id;
    active_path = full_path;
    
    for (const auto& kf_pair : bundle_adjuster->keyframes_) {
        geometry_msgs::PoseStamped pose_stamped;
        ros::Time pose_ts;
        pose_ts.fromNSec(kf_pair.second->timestamp_);
        pose_stamped.header.stamp = pose_ts;
        pose_stamped.header.frame_id = full_path.header.frame_id;
        
        // Convert the inverse pose to a geometry message.
        toGeometryMsg(pose_stamped.pose, kf_pair.second->getEigenPose().inverse());
        full_path.poses.push_back(pose_stamped);
        
        if (kf_pair.second->is_active_) {
            active_path.poses.push_back(pose_stamped);
        }
    }
    
    path_publisher.publish(full_path);
    active_path_publisher.publish(active_path);
}

} // namespace helpers
} // namespace keyframe_bundle_adjustment_ros_tool
