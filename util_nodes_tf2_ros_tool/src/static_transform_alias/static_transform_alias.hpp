#pragma once

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "util_nodes_tf2_ros_tool/StaticTransformAliasInterface.h"

namespace util_nodes_tf2_ros_tool {

class StaticTransformAlias {
public:
    // Explicit constructor to avoid implicit conversions.
    explicit StaticTransformAlias(ros::NodeHandle nh, ros::NodeHandle private_nh);

    // Delete copy constructor and assignment operator to prevent unintended copies.
    StaticTransformAlias(const StaticTransformAlias&) = delete;
    StaticTransformAlias& operator=(const StaticTransformAlias&) = delete;

private:
    // Publishers and subscribers.
    ros::Publisher dummyPub_;
    ros::Subscriber dummySub_;

    // Parameters loaded from the parameter server.
    StaticTransformAliasInterface params_;

    // Dynamic reconfigure server for runtime parameter updates.
    dynamic_reconfigure::Server<StaticTransformAliasConfig> reconfigSrv_;

    // TF components: buffer, listener, and static transform broadcaster.
    tf2_ros::Buffer tfBuffer_;
    // The transform listener must be constructed with a reference to tfBuffer_.
    tf2_ros::TransformListener tfListener_;
    tf2_ros::StaticTransformBroadcaster tfBroadcaster_;

    // Diagnostics: updater and current status.
    diagnostic_updater::Updater updater_;
    // Optionally, use a diagnosed publisher for messages with header information.
    // std::unique_ptr<diagnostic_updater::DiagnosedPublisher<std_msgs::Header>> diagnosed_pub_;
    diagnostic_msgs::DiagnosticStatus diagnosticStatus_;

    // Set up diagnostic updater and register callbacks.
    void setupDiagnostics();
    // Diagnostic callback for sensor status.
    void checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper);
    // Additional diagnostic callback for custom messages.
    void diagnostic_msg(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper);
    // Diagnose errors (if any) and update status.
    void diagnoseError();

    // Subscriber callback (using const Ptr for zero-copy transport).
    void subCallback(const std_msgs::Header::ConstPtr& msg);
    // Dynamic reconfigure callback to update parameters at runtime.
    void reconfigureRequest(StaticTransformAliasConfig& config, uint32_t level);

    // Execute aliasing: look up the transform and broadcast it.
    void do_aliasing();
};

} // namespace util_nodes_tf2_ros_tool
