#pragma once

#include <opencv2/opencv.hpp>
#include <cstdint>

namespace util_image {

/**
 * @brief Convert a color from HSV to BGR.
 *
 * The input color is in the OpenCV HSV format:
 * - H: [0, 180] (each unit represents 2°)
 * - S and V: [0, 255]
 *
 * @param colorHSV Input color in HSV.
 * @return cv::Scalar The corresponding color in BGR.
 */
inline cv::Scalar hsv2bgr(const cv::Scalar& colorHSV) {
    cv::Mat hsvMat(1, 1, CV_8UC3, colorHSV);
    cv::Mat bgrMat;
    cv::cvtColor(hsvMat, bgrMat, cv::COLOR_HSV2BGR);
    return cv::Scalar(bgrMat.at<cv::Vec3b>(0, 0));
}

/**
 * @brief Get a color based on an ID and a total number of colors.
 *
 * If ID is 0, a default color is returned. Otherwise, the function computes an HSV value
 * with the hue determined by the modulo of the (ID - 1) over the number of colors, and converts it to BGR.
 *
 * @param ID Identifier used to generate the color.
 * @param numColors Total number of distinct colors.
 * @return cv::Scalar The computed BGR color.
 */
inline cv::Scalar get_color(uint32_t ID, int numColors) {
    if (ID == 0) {
        return cv::Scalar(123, 22, 234);
    } else {
        uint32_t modID = (ID - 1) % static_cast<uint32_t>(numColors);
        int dH = 180 / numColors;
        int S = 200;
        int V = 200;
        return hsv2bgr(cv::Scalar(modID * dH, S, V));
    }
}

/**
 * @brief Get a color based solely on an ID.
 *
 * A seeded random generator ensures that the same ID always results in the same color.
 *
 * @param ID Identifier used to generate the color.
 * @return cv::Scalar A randomly generated BGR color.
 */
inline cv::Scalar get_color(uint32_t ID) {
    cv::RNG rng(ID);
    return cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
}

} // namespace util_image
