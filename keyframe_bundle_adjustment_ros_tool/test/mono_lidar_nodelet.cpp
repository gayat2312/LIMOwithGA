#include <ros/ros.h>
#include "gtest/gtest.h"

TEST(keyframe_bundle_adjustment_ros_tool, mono_lidar_test_initialization) {
    // Retrieve the initialization delay from the parameter server (default: 3 seconds)
    double init_delay = ros::NodeHandle("~").param("init_delay", 3.0);
    ros::Duration(init_delay).sleep();

    // Verify that ROS is still running after the delay.
    ASSERT_TRUE(ros::ok()) << "ROS crashed or the nodelet failed to initialize!";
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "mono_lidar_nodelet_test");

    // Use an asynchronous spinner to process callbacks while tests run.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
