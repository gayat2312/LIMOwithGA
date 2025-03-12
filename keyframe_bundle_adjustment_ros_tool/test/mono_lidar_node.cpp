#include <ros/ros.h>
#include "gtest/gtest.h"

int main(int argc, char** argv) {
    // Initialize GoogleTest.
    testing::InitGoogleTest(&argc, argv);
    
    // Initialize ROS with a node name.
    ros::init(argc, argv, "mono_lidar_test");
    
    // Start an asynchronous spinner so that callbacks are processed during tests.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    int ret = RUN_ALL_TESTS();
    
    ros::shutdown();
    return ret;
}
