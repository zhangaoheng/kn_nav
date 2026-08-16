// ============================================================================
// waypoint_utils.hpp
// ----------------------------------------------------------------------------
// PCT 全局路径 -> SCAN waypoint 路线的采样工具接口（声明）。
//
// 职责：
//   * 暴露 sampleWaypoints()：把稠密全局路径按三维弧长等间距重采样，
//     并计算路径内容签名，供 pct_scan_coordinator 判断路径是否更新。
//
// 依赖：
//   * 仅使用 nav_msgs::msg::Path，无运行时依赖，便于单元测试。
//
// 实现见 src/waypoint_utils.cpp，使用方见 src/pct_scan_coordinator.cpp。
// ============================================================================

#ifndef PCT_SCAN_NAVIGATION__WAYPOINT_UTILS_HPP_
#define PCT_SCAN_NAVIGATION__WAYPOINT_UTILS_HPP_

#include <cstdint>
#include <string>

#include <nav_msgs/msg/path.hpp>

namespace pct_scan_navigation
{

// 按三维弧长等间距重采样全局路径并计算内容签名。
// 参数：input(全局路径)、global_frame(期望坐标系)、spacing(采样间距，必须为正)、
//       z_offset(统一高度偏移)、output(重采样结果，末尾保留原终点及其朝向)、
//       signature(路径内容签名，内容不变则签名不变)、reason(失败原因)。
// 返回：路径有效且采样成功时为 true；失败时 output 为空、reason 给出原因。
bool sampleWaypoints(
    const nav_msgs::msg::Path &input,
    const std::string &global_frame,
    double spacing,
    double z_offset,
    nav_msgs::msg::Path &output,
    std::uint64_t &signature,
    std::string &reason);

}  // namespace pct_scan_navigation

#endif  // PCT_SCAN_NAVIGATION__WAYPOINT_UTILS_HPP_
