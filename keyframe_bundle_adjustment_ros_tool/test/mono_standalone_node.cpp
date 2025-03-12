#include <ros/ros.h>
#include "gtest/gtest.h"

// This test module verifies that the node (or nodelet) runs and can process messages.
// More specific functionality should be tested in dedicated unit tests of the underlying library.

int main(int argc, char** argv) {
    // Initialize GoogleTest.
    testing::InitGoogleTest(&argc, argv);
    
    // Initialize ROS.
    ros::init(argc, argv, "mono_standalone_test");
    
    // Use an asynchronous spinner to process callbacks while tests run.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    int ret = RUN_ALL_TESTS();
    
    ros::shutdown();
    return ret;
}
