#ifndef PCT_SCAN_NAVIGATION__WAYPOINT_UTILS_HPP_
#define PCT_SCAN_NAVIGATION__WAYPOINT_UTILS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include <nav_msgs/msg/path.hpp>

namespace pct_scan_navigation
{

bool sampleWaypoints(
    const nav_msgs::msg::Path &input,
    const std::string &global_frame,
    double spacing,
    double z_offset,
    nav_msgs::msg::Path &output,
    std::uint64_t &signature,
    std::string &reason);

std::size_t consumedWaypointCount(
    double accumulated_distance,
    double spacing,
    std::size_t waypoint_count);

}  // namespace pct_scan_navigation

#endif  // PCT_SCAN_NAVIGATION__WAYPOINT_UTILS_HPP_
