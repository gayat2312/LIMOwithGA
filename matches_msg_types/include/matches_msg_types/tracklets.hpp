#pragma once

#include <cstdint>
#include <vector>
#include "tracklet.hpp"

namespace matches_msg_types {

using TimestampNSec = std::uint64_t; ///< Timestamp in Unix system time (in nanoseconds)

/**
 * @brief Represents a collection of tracklets along with their corresponding timestamps.
 */
struct Tracklets {
    std::vector<TimestampNSec> stamps; ///< Timestamps associated with each track.
    std::vector<Tracklet> tracks;      ///< Collection of tracklets.
};

} // namespace matches_msg_types
