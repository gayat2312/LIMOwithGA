#include "mono_standalone.hpp"
#include <ros/ros.h>

int main(int argc, char* argv[])
{
    // Initialize ROS node with the name "mono_standalone_node"
    ros::init(argc, argv, "mono_standalone_node");

    // Create public and private node handles.
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Instantiate the MonoStandalone object.
    keyframe_bundle_adjustment_ros_tool::MonoStandalone mono_standalone(nh, nh_private);

    // Enter the ROS event loop.
    ros::spin();
    return EXIT_SUCCESS;
}
