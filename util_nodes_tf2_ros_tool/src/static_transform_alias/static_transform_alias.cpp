#include "static_transform_alias.hpp"
#include <rosinterface_handler/utilities.hpp>
#include <tf/tf.h>
#include <functional>  // For std::bind and std::placeholders

namespace util_nodes_tf2_ros_tool {

StaticTransformAlias::StaticTransformAlias(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
  : reconfigSrv_{private_node_handle},
    params_{private_node_handle},
    tfListener_{tfBuffer_} {

    // Initialization
    rosinterface_handler::setLoggerLevel(private_node_handle);
    params_.fromParamServer();
    setupDiagnostics();

    // Set up dynamic reconfiguration using std::bind (C++11)
    reconfigSrv_.setCallback(std::bind(&StaticTransformAlias::reconfigureRequest, this,
                                         std::placeholders::_1, std::placeholders::_2));

    // (Optional) Publishers & subscribers can be configured here.
    // For example, you might create diagnosed publishers and subscribers once all objects are initialized.
    // See commented-out code below for guidance:
    /*
    diagnosed_pub_ = std::make_unique<diagnostic_updater::DiagnosedPublisher<std_msgs::Header>>(
        private_node_handle.advertise<std_msgs::Header>(params_.diag_pub_msg_name, params_.msg_queue_size),
        updater_,
        diagnostic_updater::FrequencyStatusParam(&params_.diagnostic_updater_rate,
                                                 &params_.diagnostic_updater_rate,
                                                 params_.diagnostic_updater_rate_tolerance, 5),
        diagnostic_updater::TimeStampStatusParam());
    dummyPub_ = private_node_handle.advertise<std_msgs::Header>(params_.publisher_msg_name, params_.msg_queue_size);
    dummySub_ = private_node_handle.subscribe(params_.subscriber_msg_name, params_.msg_queue_size,
                                               &StaticTransformAlias::subCallback, this,
                                               ros::TransportHints().tcpNoDelay());
    */

    do_aliasing();

    rosinterface_handler::showNodeInfo();
}

void StaticTransformAlias::do_aliasing() {
    ROS_DEBUG_STREAM("Looking up transform from " << params_.from_target_frame_id << " to "
                                                    << params_.to_target_frame_id
                                                    << " with timeout " << params_.timeout << " sec");

    std::string frame_name_target, frame_name_source;
    tf::resolve(params_.from_target_frame_id, frame_name_target);
    tf::resolve(params_.from_source_frame_id, frame_name_source);

    try {
        // Attempt to look up the transform with a timeout
        geometry_msgs::TransformStamped transform =
            tfBuffer_.lookupTransform(frame_name_target, frame_name_source,
                                      ros::Time(0), ros::Duration(params_.timeout));
        // Update frame names according to alias parameters
        transform.header.frame_id = params_.to_target_frame_id;
        transform.child_frame_id = params_.to_source_frame_id;
        tfBroadcaster_.sendTransform(transform);
    }
    catch (const tf2::TransformException& ex) {
        ROS_WARN_STREAM("Transform lookup failed: " << ex.what());
    }
}

/*
// Use const Ptr for your callbacks to guarantee zero-copy transportation.
// void StaticTransformAlias::subCallback(const std_msgs::Header::ConstPtr& msg) {
//     std_msgs::Header new_msg = *msg;
//     dummyPub_.publish(new_msg);
//     // diagnosed_pub_->publish(new_msg);
//
//     diagnosticStatus_.message = "Valid loop";
//     diagnosticStatus_.level = diagnostic_msgs::DiagnosticStatus::OK;
//     updater_.update(); // Updates all registered diagnostic callbacks
// }
*/

/**
 * Callback for dynamic reconfigure changes.
 */
void StaticTransformAlias::reconfigureRequest(StaticTransformAliasConfig& config, uint32_t level) {
    params_.fromConfig(config);
    do_aliasing();
}

/**
 * Setup the diagnostic updater.
 */
void StaticTransformAlias::setupDiagnostics() {
    // Assign a unique hardware ID for diagnostics
    diagnosticStatus_.hardware_id = params_.diagnostic_updater_hardware_id;
    diagnosticStatus_.message = "Starting...";
    diagnosticStatus_.level = diagnostic_msgs::DiagnosticStatus::STALE;
    updater_.setHardwareID(params_.diagnostic_updater_hardware_id);

    // Register a diagnostic callback to check sensor status regularly
    updater_.add("StaticTransformAlias Sensor Status", this, &StaticTransformAlias::checkSensorStatus);

    updater_.force_update();
}

void StaticTransformAlias::checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper) {
    status_wrapper.summary(diagnosticStatus_);
}

} // namespace util_nodes_tf2_ros_tool
