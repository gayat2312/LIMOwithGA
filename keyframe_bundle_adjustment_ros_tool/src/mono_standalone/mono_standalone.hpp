#pragma once

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <matches_msg_ros/MatchesMsg.h>
#include <matches_msg_depth_ros/MatchesMsgWithOutlierFlag.h>
#include <sensor_msgs/CameraInfo.h>

#include <keyframe_bundle_adjustment/bundle_adjuster_keyframes.hpp>
#include <keyframe_bundle_adjustment/keyframe_selector.hpp>
#include "keyframe_bundle_adjustment_ros_tool/MonoStandaloneInterface.h"

// Forward declarations in the keyframe_bundle_adjustment namespace.
namespace keyframe_bundle_adjustment {
    class BundleAdjusterKeyframes;
    class KeyframeSelector;
}

namespace keyframe_bundle_adjustment_ros_tool {

class MonoStandalone {
public:
    /**
     * @brief Constructor: Initializes the node with public and private node handles.
     * @param nh Public node handle.
     * @param nh_private Private node handle.
     */
    MonoStandalone(ros::NodeHandle nh, ros::NodeHandle nh_private);

    /**
     * @brief Publishes the last keyframe pose as a TF transform if TF frame IDs are set.
     * @param timestamp Timestamp at which the transform should be published.
     * @param pose The pose to be published.
     */
    void maybeSendPoseTf(ros::Time timestamp, Eigen::Isometry3d pose);

private:
    // ROS-related functions.
    void setupDiagnostics();
    void checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper);
    void callbackSubscriber(const matches_msg_depth_ros::MatchesMsgWithOutlierFlag::ConstPtr& tracklets_msg,
                            const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
    void reconfigureRequest(const MonoStandaloneConfig&, uint32_t);

    // Interface and dynamic reconfigure server.
    MonoStandaloneInterface interface_;
    dynamic_reconfigure::Server<MonoStandaloneConfig> reconfigure_server_;

    // TF components.
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Message synchronization.
    using TrackletsMsg = matches_msg_depth_ros::MatchesMsgWithOutlierFlag;
    using CameraInfoMsg = sensor_msgs::CameraInfo;
    using ApproximateTime = message_filters::sync_policies::ApproximateTime<TrackletsMsg, CameraInfoMsg>;
    using Synchronizer = message_filters::Synchronizer<ApproximateTime>;
    std::unique_ptr<Synchronizer> sync_;

    // Diagnostics updater.
    diagnostic_updater::Updater updater_;
    diagnostic_msgs::DiagnosticStatus diagnostic_status_;

    // Bundle adjustment components.
    keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster_;
    keyframe_bundle_adjustment::KeyframeSelector keyframe_selector_;

    // Extrinsic calibration: transform from vehicle to camera.
    Eigen::Isometry3d trf_camera_vehicle;

    // Timestamp of the last bundle adjustment solution; negative value forces an early run.
    double last_ts_solved_{-10000000000};
};

} // namespace keyframe_bundle_adjustment_ros_tool
