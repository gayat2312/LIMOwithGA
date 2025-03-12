#pragma once

#include <cv.hpp>
#include <keyframe_bundle_adjustment/bundle_adjuster_keyframes.hpp>
#include <matches_msg_types/tracklets.hpp>
#include <commons/color_by_index_hsv.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/eigen.hpp> // attention, eigen must be included before that!
#include <sstream>
#include <set>
#include <tuple>
#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <iostream>

namespace keyframe_bundle_adjustment_ros_tool {

using CvPoints = std::vector<cv::Point2d>;

namespace helpers {

/**
 * @brief Converts a 4x4 pose matrix into a space-separated string.
 * 
 * @param m A 4x4 pose matrix.
 * @return std::string The pose represented as a string.
 */
inline std::string poseToString(const Eigen::Matrix<double, 4, 4>& m) {
    std::ostringstream ss;
    ss << m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << " " << m(0, 3) << " "
       << m(1, 0) << " " << m(1, 1) << " " << m(1, 2) << " " << m(1, 3) << " "
       << m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << " " << m(2, 3);
    return ss.str();
}

/**
 * @brief Loads a set of integer labels from a YAML file.
 * 
 * @param yaml_path Path to the YAML file.
 * @param field_name Name of the field containing a sequence of integers.
 * @return std::set<int> The set of labels.
 * @throws std::runtime_error if the field is not defined or not a sequence.
 */
inline std::set<int> loadSetFromYaml(const std::string& yaml_path, const std::string& field_name) {
    std::set<int> out;
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (root[field_name] && root[field_name].IsSequence()) {
        for (const auto& el : root[field_name]) {
            out.insert(el.as<int>());
        }
    } else {
        throw std::runtime_error("LabelReader: vector outlier_labels not defined.");
    }
    return out;
}

/**
 * @brief Retrieves matching feature points between two timestamps from tracklets.
 * 
 * It returns two sets of 2D points corresponding to the two timestamps. Outlier tracks are ignored.
 * 
 * @param tracklets The set of tracklets.
 * @param ts0 Timestamp for the first frame.
 * @param ts1 Timestamp for the second frame.
 * @param outlier_labels Set of labels to be excluded.
 * @return std::tuple<CvPoints, CvPoints> A tuple containing points for the first and second frames.
 */
inline std::tuple<CvPoints, CvPoints> getMatches(const matches_msg_types::Tracklets& tracklets,
                                                   keyframe_bundle_adjustment::TimestampNSec ts0,
                                                   keyframe_bundle_adjustment::TimestampNSec ts1,
                                                   const std::set<int>& outlier_labels) {
    // Find indices corresponding to the provided timestamps.
    int index0 = std::distance(tracklets.stamps.cbegin(),
                               std::find(tracklets.stamps.cbegin(), tracklets.stamps.cend(), ts0));
    int index1 = std::distance(tracklets.stamps.cbegin(),
                               std::find(tracklets.stamps.cbegin(), tracklets.stamps.cend(), ts1));

    CvPoints points0, points1;
    for (const auto& track : tracklets.tracks) {
        if (static_cast<int>(track.feature_points.size()) > index0 &&
            static_cast<int>(track.feature_points.size()) > index1 &&
            outlier_labels.find(track.label) == outlier_labels.end()) {
            points0.push_back(cv::Point2d(track.feature_points[index0].u, track.feature_points[index0].v));
            points1.push_back(cv::Point2d(track.feature_points[index1].u, track.feature_points[index1].v));
        }
    }
    return std::make_tuple(points0, points1);
}

/**
 * @brief Computes the mean flow (average Euclidean distance) between two sets of points.
 * 
 * @param points0 Points from the first frame.
 * @param points1 Points from the second frame.
 * @return double The mean flow.
 * @throws std::runtime_error if the sizes of the input point sets are inconsistent.
 */
inline double getMeanFlow(const CvPoints& points0, const CvPoints& points1) {
    if (points0.size() != points1.size()) {
        throw std::runtime_error("In getMeanFlow: points size not consistent.");
    }
    if (points0.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < points0.size(); ++i) {
        sum += cv::norm(points0[i] - points1[i]);
    }
    return sum / static_cast<double>(points0.size());
}

/**
 * @brief Calculates the motion between two frames using OpenCV's 5-point algorithm.
 * 
 * It uses RANSAC to compute the essential matrix and recovers the relative pose.
 * 
 * @param motion Output: the computed motion transformation.
 * @param points0 Points from the first frame.
 * @param points1 Points from the second frame.
 * @param focal Camera focal length.
 * @param pp Camera principal point.
 * @param probability RANSAC probability threshold.
 * @param flow_thres Minimum required mean flow.
 * @return true if the motion is successfully computed, false otherwise.
 */
inline bool calcMotion5Point(Eigen::Isometry3d& motion,
                             const CvPoints& points0,
                             const CvPoints& points1,
                             double focal,
                             cv::Point2d pp,
                             double probability,
                             double flow_thres = 3.0) {
    if (points0.empty() || points1.empty()) {
        ROS_DEBUG_STREAM("No points for 5-point algorithm available!");
        return false;
    }
    if (getMeanFlow(points0, points1) < flow_thres) {
        motion.translation() = Eigen::Vector3d::Zero();
        ROS_DEBUG_STREAM("Not enough flow; returning motion=\n" << motion.matrix());
        return false;
    }
    cv::Mat essential_mat = cv::findEssentialMat(points1, points0, focal, pp, cv::RANSAC, probability, 2.0);
    if (essential_mat.rows != 3 || essential_mat.cols != 3) {
        return false;
    }
    cv::Mat R, t;
    int number_inliers = cv::recoverPose(essential_mat, points1, points0, R, t, focal, pp);
    Eigen::Matrix3d r_eigen;
    Eigen::Vector3d t_eigen;
    cv::cv2eigen(R, r_eigen);
    cv::cv2eigen(t, t_eigen);
    motion.linear() = r_eigen;
    motion.translation() = t_eigen;
    return true;
}

/**
 * @brief Generates a debug image visualizing flow by drawing circles at measurement locations.
 * 
 * @param bundle_adjuster Pointer to the bundle adjuster containing keyframes.
 * @return cv::Mat Debug image with drawn feature points.
 */
inline cv::Mat getFlowImg(keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster) {
    cv::Mat debug_img(600, 1300, CV_8UC3, cv::Scalar(0, 0, 0));
    int num_colors = 10;
    for (const auto& kf : bundle_adjuster->keyframes_) {
        if (kf.second->is_active_) {
            for (const auto& m : kf.second->measurements_) {
                const auto& m_id = m.first;
                const auto& meas = m.second.cbegin()->second;
                auto color = util_image::get_color(m_id, num_colors);
                cv::circle(debug_img, cv::Point(static_cast<int>(meas.u), static_cast<int>(meas.v)), 1, color, -1);
            }
        }
    }
    return debug_img;
}

/**
 * @brief Dumps the current map (landmarks and poses) into a YAML-like file.
 * 
 * @param filename The output file path.
 * @param bundle_adjuster Pointer to the bundle adjuster containing map information.
 * @throws std::runtime_error if the file cannot be opened.
 */
inline void dumpMap(const std::string& filename,
                    const keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr& bundle_adjuster) {
    std::ofstream file(filename);
    if (!file) {
        throw std::runtime_error("Cannot open file: " + filename);
    }
    file.precision(12);

    file << "landmarks with depth: [";
    for (const auto& id_lm_ptr : bundle_adjuster->landmarks_) {
        if (id_lm_ptr.second->has_measured_depth) {
            file << "[" << id_lm_ptr.second->pos[0] << ", " << id_lm_ptr.second->pos[1]
                 << ", " << id_lm_ptr.second->pos[2] << ", " << id_lm_ptr.second->weight << "],\n";
        }
    }
    file << "]\n";

    file << "landmarks without depth: [";
    for (const auto& id_lm_ptr : bundle_adjuster->landmarks_) {
        if (!id_lm_ptr.second->has_measured_depth) {
            file << "[" << id_lm_ptr.second->pos[0] << ", " << id_lm_ptr.second->pos[1]
                 << ", " << id_lm_ptr.second->pos[2] << ", " << id_lm_ptr.second->weight << "],\n";
        }
    }
    file << "]\n";

    file << "poses: {";
    for (const auto& id_kf_ptr : bundle_adjuster->keyframes_) {
        file << id_kf_ptr.second->timestamp_ << ": [" 
             << id_kf_ptr.second->pose_[0] << ", " << id_kf_ptr.second->pose_[1] << ", "
             << id_kf_ptr.second->pose_[2] << ", " << id_kf_ptr.second->pose_[3] << ", "
             << id_kf_ptr.second->pose_[4] << ", " << id_kf_ptr.second->pose_[5] << ", "
             << id_kf_ptr.second->pose_[6] << "],\n";
    }
    file << "}";
    file.close();

    std::cout << "--------------------------------------\nDumped map to " << filename
              << "\n--------------------------------------\n" << std::endl;
}

/**
 * @brief Computes the unscaled motion between two timestamps using feature tracklets.
 * 
 * This function computes the camera motion using the 5-point algorithm and then scales the translation
 * based on a provided vehicle speed and the time difference between frames. Finally, it converts the motion
 * from the camera coordinate system to the vehicle coordinate system.
 * 
 * @param focal_length Camera focal length.
 * @param princ_point Camera principal point.
 * @param cur_ts Timestamp for the current frame.
 * @param ts_last_kf Timestamp for the last keyframe.
 * @param tracklets The tracklets containing feature point measurements.
 * @param trf_camera_vehicle Transformation from the camera to the vehicle coordinate system.
 * @param speed_m_per_second Vehicle speed in m/s (default is 13.0 m/s).
 * @return Eigen::Isometry3d The computed motion transformation from the vehicle at time t1 to time t0.
 */
inline Eigen::Isometry3d getMotionUnscaled(double focal_length,
                                           cv::Point2d princ_point,
                                           const keyframe_bundle_adjustment::TimestampNSec& cur_ts,
                                           const keyframe_bundle_adjustment::TimestampNSec& ts_last_kf,
                                           const keyframe_bundle_adjustment::Tracklets& tracklets,
                                           const Eigen::Isometry3d& trf_camera_vehicle,
                                           double speed_m_per_second = 13.0) {
    CvPoints last_points, cur_points;
    std::tie(last_points, cur_points) = getMatches(tracklets, ts_last_kf, cur_ts, {23, 24, 25, 26});

    Eigen::Isometry3d motion_camera_t0_t1 = Eigen::Isometry3d::Identity();
    motion_camera_t0_t1.translation() = Eigen::Vector3d(0., 0., 1.);
    calcMotion5Point(motion_camera_t0_t1, last_points, cur_points, focal_length, princ_point, 0.999, 5.0);

    double dt = keyframe_bundle_adjustment::convert(cur_ts) - keyframe_bundle_adjustment::convert(ts_last_kf);
    double norm_trans = std::max(0.0001, motion_camera_t0_t1.translation().norm());
    motion_camera_t0_t1.translation() = motion_camera_t0_t1.translation() / norm_trans *
                                         speed_m_per_second * dt;

    Eigen::Isometry3d motion_vehicle_t1_t0 =
        trf_camera_vehicle.inverse() * motion_camera_t0_t1.inverse() * trf_camera_vehicle;
    return motion_vehicle_t1_t0;
}

} // namespace helpers
} // namespace keyframe_bundle_adjustment_ros_tool
