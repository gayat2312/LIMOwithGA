// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
// Basic assertions:
//   ASSERT_EQ(expected, actual);  // Fatal assertion: stops test on failure.
//   EXPECT_EQ(expected, actual);  // Nonfatal assertion: test continues on failure.
//
// Floating point comparisons:
//   ASSERT_FLOAT_EQ(expected, actual);  // Nearly equal floats (4 ULPs)
//   ASSERT_DOUBLE_EQ(expected, actual); // Nearly equal doubles (4 ULPs)
//   ASSERT_NEAR(val1, val2, abs_error);   // Checks that absolute difference does not exceed abs_error
//
// Example tests:
//
// TEST(Math, Add) {
//    ASSERT_EQ(10, 5 + 5);
// }
//
// TEST(Math, Float) {
//    ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f);
// }

#include <ros/ros.h>
#include "gtest/gtest.h"

TEST(keyframe_bundle_adjustment_ros_tool, mono_standalone_test_initialization) {
    // Retrieve initialization delay from parameter server (default: 3 seconds)
    auto init_delay = ros::NodeHandle("~").param("init_delay", 3.0);
    ros::Duration(init_delay).sleep();

    // Ensure that ROS remains running after the delay.
    ASSERT_TRUE(ros::ok()) << "ROS crashed or the nodelet failed to initialize!";
}

int main(int argc, char** argv) {
    // Initialize GoogleTest.
    testing::InitGoogleTest(&argc, argv);
    // Initialize ROS.
    ros::init(argc, argv, "mono_standalone_nodelet_test");

    // Use an asynchronous spinner so that callbacks are processed during tests.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int ret = RUN_ALL_TESTS();

    ros::shutdown();
    return ret;
}
