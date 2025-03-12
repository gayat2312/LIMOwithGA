// Updated GoogleTest Documentation:
// Official website: https://google.github.io/googletest/
// GitHub repository: https://github.com/google/googletest
//
// Basic assertions:
//   ASSERT_EQ(expected, actual);     // Fatal assertion: test stops if failure occurs.
//   EXPECT_EQ(expected, actual);     // Non-fatal assertion: test continues on failure.
//
// Floating-point comparisons:
//   ASSERT_FLOAT_EQ(expected, actual); // Compares floats (within 4 ULPs)
//   ASSERT_DOUBLE_EQ(expected, actual);// Compares doubles (within 4 ULPs)
//   ASSERT_NEAR(val1, val2, abs_error);  // Verifies that the difference does not exceed abs_error
//
// Example tests:

#include "gtest/gtest.h"

TEST(Math, Add) {
    // Verifies that addition is performed correctly.
    ASSERT_EQ(10, 5 + 5);
}

TEST(Math, Float) {
    // Verifies that floating point arithmetic is computed as expected.
    ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f);
}

// Note: The GoogleTest framework provides a main() function by default.
// If you wish to define your own main, you can uncomment the code below:
//
// int main(int argc, char **argv) {
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
