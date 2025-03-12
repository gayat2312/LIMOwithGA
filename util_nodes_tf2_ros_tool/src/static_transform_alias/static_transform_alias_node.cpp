#include "static_transform_alias.hpp"
#include <ros/ros.h>

int main(int argc, char* argv[]) {
    // Initialize the ROS node with a specified name.
    ros::init(argc, argv, "static_transform_alias_node");

    // Create node handles: one for public and one for private parameters.
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Instantiate the StaticTransformAlias object.
    util_nodes_tf2_ros_tool::StaticTransformAlias static_transform_alias(nh, private_nh);

    // Enter the ROS event loop.
    ros::spin();
    return 0;
}
