// Updated GoogleTest Documentation:
// Official website: https://google.github.io/googletest/
// GitHub repository: https://github.com/google/googletest
//
// Basic assertions (fatal vs. nonfatal):
//   ASSERT_EQ(expected, actual);     // Fails immediately if expected != actual
//   EXPECT_EQ(expected, actual);     // Records the failure but continues running
//
// Other useful assertions:
//   ASSERT_NE(val1, val2);           // Asserts val1 != val2
//   ASSERT_LT(val1, val2);           // Asserts val1 < val2
//   ASSERT_LE(val1, val2);           // Asserts val1 <= val2
//   ASSERT_GT(val1, val2);           // Asserts val1 > val2
//   ASSERT_GE(val1, val2);           // Asserts val1 >= val2
//
// Floating-point comparisons:
//   ASSERT_FLOAT_EQ(expected, actual); // Nearly equal floats (4 ULPs)
//   ASSERT_DOUBLE_EQ(expected, actual); // Nearly equal doubles (4 ULPs)
//   ASSERT_NEAR(val1, val2, abs_error);  // Absolute error threshold
//
// Example tests:
//
// TEST(Math, Add) {
//     ASSERT_EQ(10, 5 + 5);
// }
//
// TEST(Math, Float) {
//     ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f);
// }

#include <gtest/gtest.h>
#include <ros/ros.h>

// Sample test for integer addition
TEST(Math, Add) {
    ASSERT_EQ(10, 5 + 5);
}

// Sample test for floating-point arithmetic
TEST(Math, Float) {
    ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f);
}

int main(int argc, char **argv) {
    // Initialize ROS node for tests that might use ROS functionalities.
    ros::init(argc, argv, "unittest");

    // Initialize GoogleTest framework.
    ::testing::InitGoogleTest(&argc, argv);
    
    // Run all tests and return the result.
    return RUN_ALL_TESTS();
}
